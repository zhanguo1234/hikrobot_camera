#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "MvCameraControl.h"

class HikCameraNode : public rclcpp::Node
{
public:
    HikCameraNode() : Node("hik_camera_node")
    {
        // ---------- 参数声明 ----------
        // 注意：这里【先声明、后注册回调】。
        // 因为 declare_parameter 也会触发参数回调，而那时相机还没打开，
        // 先声明可以彻底避开"回调里操作空句柄"导致节点崩溃的问题。
        // 默认 -1 / 空字符串 表示"用户没指定，不要动相机"
        camera_serial_ = declare_parameter<std::string>("camera_serial", "");
        camera_ip_     = declare_parameter<std::string>("camera_ip", "");
        topic_name_    = declare_parameter<std::string>("topic_name", "image_raw");
        frame_id_      = declare_parameter<std::string>("frame_id", "camera");
        pixel_format_  = declare_parameter<std::string>("pixel_format", "BayerRG8");
        exposure_time_ = declare_parameter<double>("exposure_time", -1.0);
        gain_          = declare_parameter<double>("gain", -1.0);
        frame_rate_    = declare_parameter<double>("frame_rate", -1.0);

        // 发布者：话题名来自参数
        publisher_ = create_publisher<sensor_msgs::msg::Image>(topic_name_, 10);

        // 注册参数回调：运行中执行 ros2 param set 会走到 on_set_parameters
        param_cb_ = add_on_set_parameters_callback(
            std::bind(&HikCameraNode::on_set_parameters, this, std::placeholders::_1));

        int ret = MV_CC_Initialize();
        if (ret != MV_OK) {
            RCLCPP_FATAL(get_logger(), "MV_CC_Initialize 失败: 0x%X", ret);
            return;
        }

        // 打开相机（连不上也不退出，交给取流线程自动重试）
        if (open_camera()) {
            RCLCPP_INFO(get_logger(), "相机已连接: %s (序列号 %s)",
                        model_.c_str(), serial_.c_str());
            RCLCPP_INFO(get_logger(), "开始取流，发布话题 %s", topic_name_.c_str());
        } else {
            RCLCPP_WARN(get_logger(), "启动时未连上相机，之后每 2 秒自动重试");
        }

        running_ = true;
        grab_thread_ = std::thread(&HikCameraNode::grab_loop, this);
    }

    ~HikCameraNode() override
    {
        running_ = false;
        if (grab_thread_.joinable()) {
            grab_thread_.join();
        }
        close_camera();
        MV_CC_Finalize();
    }

private:
    // ==================== 像素格式 ====================
    static bool pixel_format_value(const std::string & name, unsigned int & value)
    {
        if (name == "Mono8") {
            value = PixelType_Gvsp_Mono8;
        } else if (name == "BayerRG8") {
            value = PixelType_Gvsp_BayerRG8;
        } else if (name == "BayerRG10") {
            value = PixelType_Gvsp_BayerRG10;
        } else if (name == "BayerRG12") {
            value = PixelType_Gvsp_BayerRG12;
        } else if (name == "RGB8") {
            value = PixelType_Gvsp_RGB8_Packed;
        } else if (name == "BGR8") {
            value = PixelType_Gvsp_BGR8_Packed;
        } else if (name == "YUV422_YUYV") {
            value = PixelType_Gvsp_YUV422_YUYV_Packed;
        } else if (name == "YUV422") {
            value = PixelType_Gvsp_YUV422_Packed;
        } else {
            return false;
        }
        return true;
    }

    // 切换像素格式【必须暂停采集】，否则相机直接返回错误
    std::string apply_pixel_format(const std::string & name)
    {
        unsigned int value = 0;
        if (!pixel_format_value(name, value)) {
            return "不支持的像素格式: " + name +
                   "（可选 Mono8 / BayerRG8 / BayerRG10 / BayerRG12 / RGB8 / BGR8 / YUV422_YUYV / YUV422）";
        }

        const bool was_grabbing = grabbing_;
        if (was_grabbing) {
            int ret = MV_CC_StopGrabbing(handle_);
            if (ret != MV_OK) {
                char buf[64];
                snprintf(buf, sizeof(buf), "暂停采集失败 0x%X", ret);
                return std::string(buf);
            }
            grabbing_ = false;
        }

        int ret = MV_CC_SetEnumValue(handle_, "PixelFormat", value);
        if (ret != MV_OK) {
            if (was_grabbing) {
                MV_CC_StartGrabbing(handle_);
                grabbing_ = true;
            }
            char buf[128];
            snprintf(buf, sizeof(buf), "设置像素格式失败 0x%X（该型号可能不支持 %s）",
                     ret, name.c_str());
            return std::string(buf);
        }

        if (was_grabbing) {
            ret = MV_CC_StartGrabbing(handle_);
            if (ret != MV_OK) {
                char buf[64];
                snprintf(buf, sizeof(buf), "恢复采集失败 0x%X", ret);
                return std::string(buf);
            }
            grabbing_ = true;
        }

        RCLCPP_INFO(get_logger(), "像素格式已设为 %s", name.c_str());
        return "";
    }

    // ==================== 参数下发 ====================
    // 说明：这几个函数【返回空字符串 = 成功】，非空字符串 = 失败原因
    void apply_startup_params()
    {
        std::string err;

        if (exposure_time_ > 0.0) {
            err = apply_exposure(exposure_time_);
            if (!err.empty()) {
                RCLCPP_ERROR(get_logger(), "启动参数 exposure_time 无效: %s", err.c_str());
            }
        }
        if (gain_ >= 0.0) {
            err = apply_gain(gain_);
            if (!err.empty()) {
                RCLCPP_ERROR(get_logger(), "启动参数 gain 无效: %s", err.c_str());
            }
        }
        if (frame_rate_ > 0.0) {
            err = apply_frame_rate(frame_rate_);
            if (!err.empty()) {
                RCLCPP_ERROR(get_logger(), "启动参数 frame_rate 无效: %s", err.c_str());
            }
        }
    }

    std::string apply_exposure(double value)
    {
        MVCC_FLOATVALUE range;
        memset(&range, 0, sizeof(range));
        int ret = MV_CC_GetFloatValue(handle_, "ExposureTime", &range);
        if (ret != MV_OK) {
            return "读取曝光范围失败";
        }
        if (value < range.fMin || value > range.fMax) {
            char buf[128];
            snprintf(buf, sizeof(buf), "曝光 %.1f 超出范围 [%.1f, %.1f] us",
                     value, range.fMin, range.fMax);
            return std::string(buf);
        }

        // 手动曝光之前必须先关掉自动曝光，否则写入会被相机忽略
        ret = MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);
        if (ret != MV_OK) {
            return "关闭自动曝光失败";
        }

        ret = MV_CC_SetFloatValue(handle_, "ExposureTime", (float)value);
        if (ret != MV_OK) {
            char buf[64];
            snprintf(buf, sizeof(buf), "设置曝光失败 0x%X", ret);
            return std::string(buf);
        }
        RCLCPP_INFO(get_logger(), "曝光已设为 %.1f us", value);
        return "";
    }

    std::string apply_gain(double value)
    {
        MVCC_FLOATVALUE range;
        memset(&range, 0, sizeof(range));
        int ret = MV_CC_GetFloatValue(handle_, "Gain", &range);
        if (ret != MV_OK) {
            return "读取增益范围失败";
        }
        if (value < range.fMin || value > range.fMax) {
            char buf[128];
            snprintf(buf, sizeof(buf), "增益 %.1f 超出范围 [%.1f, %.1f] dB",
                     value, range.fMin, range.fMax);
            return std::string(buf);
        }

        ret = MV_CC_SetEnumValue(handle_, "GainAuto", 0);
        if (ret != MV_OK) {
            return "关闭自动增益失败";
        }

        ret = MV_CC_SetFloatValue(handle_, "Gain", (float)value);
        if (ret != MV_OK) {
            char buf[64];
            snprintf(buf, sizeof(buf), "设置增益失败 0x%X", ret);
            return std::string(buf);
        }
        RCLCPP_INFO(get_logger(), "增益已设为 %.1f dB", value);
        return "";
    }

    std::string apply_frame_rate(double value)
    {
        MVCC_FLOATVALUE range;
        memset(&range, 0, sizeof(range));
        int ret = MV_CC_GetFloatValue(handle_, "AcquisitionFrameRate", &range);
        if (ret != MV_OK) {
            return "读取帧率范围失败";
        }
        if (value < range.fMin || value > range.fMax) {
            char buf[128];
            snprintf(buf, sizeof(buf), "帧率 %.2f 超出范围 [%.2f, %.2f] fps",
                     value, range.fMin, range.fMax);
            return std::string(buf);
        }

        // 必须先把"帧率控制开关"打开，帧率设置才会生效
        ret = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
        if (ret != MV_OK) {
            RCLCPP_WARN(get_logger(), "AcquisitionFrameRateEnable 失败 0x%X，仍尝试写入", ret);
        }

        ret = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", (float)value);
        if (ret != MV_OK) {
            char buf[64];
            snprintf(buf, sizeof(buf), "设置帧率失败 0x%X", ret);
            return std::string(buf);
        }
        RCLCPP_INFO(get_logger(), "帧率已设为 %.2f fps", value);
        return "";
    }

    // ==================== 运行中改参数的回调 ====================
    rcl_interfaces::msg::SetParametersResult on_set_parameters(
        const std::vector<rclcpp::Parameter> & params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        // 相机没连上时，跟相机有关的参数一律拒绝，并说明原因
        if (handle_ == nullptr) {
            result.successful = false;
            result.reason = "相机未连接（正在自动重连，请稍后再试）";
            return result;
        }

        // 先"举旗"：告诉取流线程 ROS 线程要用相机了，请让一让。
        // 取流线程在 70fps 的紧循环里会不停重新抢锁，而 std::mutex 不保证公平，
        // 没有这个让步机制，本回调会被无限期饿死
        //（现象：ros2 param set 永远不返回，之后连 param list 也没反应）。
        param_pending_ = true;
        std::unique_lock<std::mutex> lock(sdk_mutex_);
        param_pending_ = false;

        for (const auto & p : params) {
            const std::string & name = p.get_name();
            std::string err;

            if (name == "exposure_time") {
                const double v = p.as_double();
                if (v <= 0.0) {
                    continue;                  // 负值/0 = "不指定"，跳过
                }
                err = apply_exposure(v);
                if (err.empty()) {
                    exposure_time_ = v;        // 记住，重连后要恢复
                }
            } else if (name == "gain") {
                const double v = p.as_double();
                if (v < 0.0) {
                    continue;
                }
                err = apply_gain(v);
                if (err.empty()) {
                    gain_ = v;
                }
            } else if (name == "frame_rate") {
                const double v = p.as_double();
                if (v <= 0.0) {
                    continue;
                }
                err = apply_frame_rate(v);
                if (err.empty()) {
                    frame_rate_ = v;
                }
            } else if (name == "pixel_format") {
                err = apply_pixel_format(p.as_string());
                if (err.empty()) {
                    pixel_format_ = p.as_string();
                }
            } else if (name == "frame_id") {
                frame_id_ = p.as_string();
                continue;
            } else if (name == "topic_name") {
                topic_name_ = p.as_string();
                RCLCPP_WARN(get_logger(), "topic_name 需要重启节点才能生效");
                continue;
            } else if (name == "camera_serial" || name == "camera_ip") {
                RCLCPP_WARN(get_logger(), "%s 需要重启节点才能生效", name.c_str());
                continue;
            } else {
                continue;   // 其他参数（use_sim_time 等）不归我们管
            }

            if (!err.empty()) {
                result.successful = false;
                result.reason = name + ": " + err;   // 这句话会原样打印给用户
                return result;
            }
        }
        return result;
    }

    // ==================== 相机连接 ====================
    bool open_camera()
    {
        MV_CC_DEVICE_INFO_LIST list;
        memset(&list, 0, sizeof(list));
        int ret = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &list);
        if (ret != MV_OK || list.nDeviceNum == 0) {
            RCLCPP_WARN(get_logger(), "枚举不到相机 (0x%X, 数量=%u)", ret, list.nDeviceNum);
            return false;
        }

        // 按 序列号 / IP 匹配；两个都没指定就用第一台
        MV_CC_DEVICE_INFO * target = nullptr;
        for (unsigned int i = 0; i < list.nDeviceNum && target == nullptr; ++i) {
            MV_CC_DEVICE_INFO * info = list.pDeviceInfo[i];
            if (info == nullptr) {
                continue;
            }

            std::string model, serial, ip;
            if (info->nTLayerType == MV_USB_DEVICE) {
                model  = (char *)info->SpecialInfo.stUsb3VInfo.chModelName;
                serial = (char *)info->SpecialInfo.stUsb3VInfo.chSerialNumber;
            } else if (info->nTLayerType == MV_GIGE_DEVICE) {
                model  = (char *)info->SpecialInfo.stGigEInfo.chModelName;
                serial = (char *)info->SpecialInfo.stGigEInfo.chSerialNumber;
                unsigned int raw = info->SpecialInfo.stGigEInfo.nCurrentIp;
                char buf[32];
                snprintf(buf, sizeof(buf), "%u.%u.%u.%u",
                         (raw >> 24) & 0xFF, (raw >> 16) & 0xFF,
                         (raw >> 8) & 0xFF, raw & 0xFF);
                ip = buf;
            } else {
                continue;
            }

            RCLCPP_INFO(get_logger(), "发现相机: 型号=%s 序列号=%s IP=%s",
                        model.c_str(), serial.c_str(),
                        ip.empty() ? "-" : ip.c_str());

            if (!camera_serial_.empty()) {
                if (serial == camera_serial_) {
                    target = info;
                }
            } else if (!camera_ip_.empty()) {
                if (ip == camera_ip_) {
                    target = info;
                }
            } else {
                target = info;
            }

            if (target != nullptr) {
                model_  = model;
                serial_ = serial;
            }
        }

        if (target == nullptr) {
            RCLCPP_ERROR(get_logger(), "没有匹配 camera_serial='%s' / camera_ip='%s' 的相机",
                         camera_serial_.c_str(), camera_ip_.c_str());
            return false;
        }

        ret = MV_CC_CreateHandle(&handle_, target);
        if (ret != MV_OK) {
            RCLCPP_ERROR(get_logger(), "MV_CC_CreateHandle 失败: 0x%X", ret);
            handle_ = nullptr;
            return false;
        }

        ret = MV_CC_OpenDevice(handle_, MV_ACCESS_Exclusive, 0);
        if (ret != MV_OK) {
            RCLCPP_ERROR(get_logger(), "MV_CC_OpenDevice 失败: 0x%X", ret);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
            return false;
        }

        // 连上后立刻恢复全部已配置参数（重连成功后同样走这里）
        apply_startup_params();
        std::string err = apply_pixel_format(pixel_format_);
        if (!err.empty()) {
            RCLCPP_ERROR(get_logger(), "恢复 pixel_format 失败: %s", err.c_str());
        }

        ret = MV_CC_StartGrabbing(handle_);
        if (ret != MV_OK) {
            RCLCPP_ERROR(get_logger(), "MV_CC_StartGrabbing 失败: 0x%X", ret);
            close_camera_locked();
            return false;
        }
        grabbing_ = true;
        no_frame_count_ = 0;
        return true;
    }

    // 调用者必须已持有 sdk_mutex_
    void close_camera_locked()
    {
        if (handle_ == nullptr) {
            return;
        }
        if (grabbing_) {
            MV_CC_StopGrabbing(handle_);
            grabbing_ = false;
        }
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
    }

    void close_camera()
    {
        std::lock_guard<std::mutex> lock(sdk_mutex_);
        close_camera_locked();
    }

    // ==================== 取流（含断线重连） ====================
    void grab_loop()
    {
        MV_FRAME_OUT frame;

        while (running_) {
            // 如果 ROS 线程正在等这把锁，先让它拿到，避免它被这个紧循环饿死
            while (param_pending_ && running_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            {
                std::lock_guard<std::mutex> lock(sdk_mutex_);
                if (!running_) {
                    break;
                }

                if (handle_ == nullptr) {
                    // ---- 掉线状态：尝试重连 ----
                    if (open_camera()) {
                        RCLCPP_INFO(get_logger(), "相机已重新连接，参数已恢复");
                        continue;
                    }
                } else {
                    memset(&frame, 0, sizeof(frame));
                    int ret = MV_CC_GetImageBuffer(handle_, &frame, 200);

                    if (ret == MV_OK) {
                        no_frame_count_ = 0;
                        publish_frame(frame);
                        MV_CC_FreeImageBuffer(handle_, &frame);
                        continue;
                    }

                    if (static_cast<unsigned int>(ret) == MV_E_NODATA) {
                        // 超时本身是正常的；但长时间一帧都没有，说明设备已经掉了
                        //（USB 被拔掉时 SDK 常常一直报超时而不报错）
                        if (++no_frame_count_ > kNoFrameLimit) {
                            RCLCPP_WARN(get_logger(),
                                        "连续 %d 次收不到图像，判定为掉线，开始重连",
                                        no_frame_count_);
                            close_camera_locked();
                            no_frame_count_ = 0;
                        }
                    } else {
                        RCLCPP_ERROR(get_logger(), "取流失败 0x%X，判定为掉线，开始重连", ret);
                        close_camera_locked();
                        no_frame_count_ = 0;
                    }
                }
            }

            // 没连上时，等 2 秒再试（分片睡眠，保证能及时退出）
            if (handle_ == nullptr) {
                for (int i = 0; i < 20 && running_; ++i) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        }
    }

    void publish_frame(const MV_FRAME_OUT & frame)
    {
        const MV_FRAME_OUT_INFO_EX & info = frame.stFrameInfo;

        const unsigned char * data     = frame.pBufAddr;
        size_t                length   = info.nFrameLen;
        const char *          encoding = nullptr;
        size_t                step     = 0;

        if (info.enPixelType == PixelType_Gvsp_Mono8) {
            encoding = "mono8";
            step = info.nWidth;
        } else if (info.enPixelType == PixelType_Gvsp_BGR8_Packed) {
            encoding = "bgr8";
            step = (size_t)info.nWidth * 3;
        } else if (info.enPixelType == PixelType_Gvsp_RGB8_Packed) {
            encoding = "rgb8";
            step = (size_t)info.nWidth * 3;
        } else {
            // 相机给的不是 ROS 能直接用的格式（Bayer / YUV 等），用 SDK 转成 BGR8
            const size_t need = (size_t)info.nWidth * info.nHeight * 3;
            if (convert_buf_.size() < need) {
                convert_buf_.resize(need);
            }

            MV_CC_PIXEL_CONVERT_PARAM_EX cvt;
            memset(&cvt, 0, sizeof(cvt));
            cvt.nWidth         = info.nWidth;
            cvt.nHeight        = info.nHeight;
            cvt.enSrcPixelType = info.enPixelType;
            cvt.pSrcData       = frame.pBufAddr;
            cvt.nSrcDataLen    = info.nFrameLen;
            cvt.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
            cvt.pDstBuffer     = convert_buf_.data();
            cvt.nDstBufferSize = (unsigned int)convert_buf_.size();

            int ret = MV_CC_ConvertPixelTypeEx(handle_, &cvt);
            if (ret != MV_OK) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                      "像素格式转换失败 0x%X (源格式 0x%X)",
                                      ret, (unsigned int)info.enPixelType);
                return;
            }

            data     = convert_buf_.data();
            length   = cvt.nDstLen;
            encoding = "bgr8";
            step     = (size_t)info.nWidth * 3;
        }

        sensor_msgs::msg::Image msg;
        msg.header.stamp = now();
        msg.header.frame_id = frame_id_;
        msg.height = info.nHeight;
        msg.width = info.nWidth;
        msg.encoding = encoding;
        msg.is_bigendian = false;
        msg.step = (sensor_msgs::msg::Image::_step_type)step;
        msg.data.assign(data, data + length);

        publisher_->publish(msg);
    }

    // ==================== 成员 ====================
    static constexpr int kNoFrameLimit = 25;   // 25 × 200ms = 5 秒没图就判定掉线

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    void * handle_{nullptr};
    bool grabbing_{false};
    int no_frame_count_{0};
    std::mutex sdk_mutex_;          // 保护对相机句柄的并发访问
    std::atomic<bool> param_pending_{false};   // ROS 线程正在等锁的标记
    std::string model_;
    std::string serial_;

    std::string camera_serial_;
    std::string camera_ip_;
    std::string topic_name_;
    std::string frame_id_;
    std::string pixel_format_;
    double exposure_time_{-1.0};
    double gain_{-1.0};
    double frame_rate_{-1.0};

    std::vector<unsigned char> convert_buf_;
    std::atomic<bool> running_{false};
    std::thread grab_thread_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HikCameraNode>());
    rclcpp::shutdown();
    return 0;
}
