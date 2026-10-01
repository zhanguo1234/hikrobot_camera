"""启动海康相机节点。

用法：
    ros2 launch hikrobot_camera hik_camera.launch.py
    ros2 launch hikrobot_camera hik_camera.launch.py camera_serial:=DB0178696 exposure_time:=5000.0
"""

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 海康 MVS SDK 的库目录
MVS_LIB_DIR = '/opt/MVS/lib/64'


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'camera_serial', default_value='',
            description='相机序列号；留空则配合 camera_ip，两者都留空时连接第一台'),
        DeclareLaunchArgument(
            'camera_ip', default_value='',
            description='GigE 相机 IP'),
        DeclareLaunchArgument(
            'topic_name', default_value='image_raw',
            description='图像发布话题（修改后需重启节点）'),
        DeclareLaunchArgument(
            'frame_id', default_value='camera',
            description='图像消息的 frame_id'),
        DeclareLaunchArgument(
            'pixel_format', default_value='BayerRG8',
            description='Mono8 / BayerRG8 / BayerRG10 / BayerRG12 / '
                        'RGB8 / BGR8 / YUV422_YUYV / YUV422'),
        DeclareLaunchArgument(
            'exposure_time', default_value='-1.0',
            description='曝光时间 us；负值表示不修改相机设置'),
        DeclareLaunchArgument(
            'gain', default_value='-1.0',
            description='增益 dB；负值表示不修改相机设置'),
        DeclareLaunchArgument(
            'frame_rate', default_value='-1.0',
            description='采集帧率 fps；负值表示不修改相机设置'),
    ]

    # 这一条很重要：MVS SDK 在做像素格式转换时会 dlopen
    # /opt/MVS/lib/64/libFormatConversion.so，而 dlopen 不认可执行文件上的
    # RUNPATH，只能靠 LD_LIBRARY_PATH。缺了它 Bayer 之类的格式会报
    # MV_E_LOAD_LIBRARY (0x8000000C)。
    mvs_env = AppendEnvironmentVariable('LD_LIBRARY_PATH', MVS_LIB_DIR)

    node = Node(
        package='hikrobot_camera',
        executable='hik_camera_node',
        name='hik_camera_node',
        output='screen',
        parameters=[{
            'camera_serial': LaunchConfiguration('camera_serial'),
            'camera_ip': LaunchConfiguration('camera_ip'),
            'topic_name': LaunchConfiguration('topic_name'),
            'frame_id': LaunchConfiguration('frame_id'),
            'pixel_format': LaunchConfiguration('pixel_format'),
            'exposure_time': LaunchConfiguration('exposure_time'),
            'gain': LaunchConfiguration('gain'),
            'frame_rate': LaunchConfiguration('frame_rate'),
        }],
    )

    return LaunchDescription(args + [mvs_env, node])
