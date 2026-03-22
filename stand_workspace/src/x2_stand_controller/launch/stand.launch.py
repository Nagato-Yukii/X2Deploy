import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 科学的第一步：动态获取包的 share 目录路径
    package_name = 'x2_stand_controller'
    pkg_share = get_package_share_directory(package_name)

    # 精准定位你的核心资产：yaml 配置文件
    config_file = os.path.join(pkg_share, 'config', 'initial_pose.yaml')

    # 实例化节点，并将参数文件注入
    stand_node = Node(
        package=package_name,
        executable='stand_node',  # 对应 CMakeLists.txt 里 add_executable 的名字
        name='stand_node',
        output='screen',          # 让终端能打印出那句 El Psy Kongroo
        parameters=[config_file]  # 极其关键：加载静态位姿参数
    )

    return LaunchDescription([
        stand_node
    ])