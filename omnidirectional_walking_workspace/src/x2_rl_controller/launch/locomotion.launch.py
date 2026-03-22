import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 科学的第一步：动态获取包的 share 目录路径
    package_name = 'x2_rl_controller'
    pkg_share = get_package_share_directory(package_name)

    # 精准定位你的核心资产：全向行走模式的 yaml 配置文件
    config_file = os.path.join(pkg_share, 'config', 'locomotion.yaml')

    # 实例化核心的 RL 闭环控制节点
    rl_node = Node(
        package=package_name,
        executable='rl_node',      # 对应 CMakeLists.txt 里 add_executable 的名字
        name='rl_locomotion_node', # 必须和 rl_node.cpp 里 Node("...") 的名字一致
        output='screen',           # 确保终端能打印出 "El Psy Kongroo"
        parameters=[config_file]   # 极其关键：将 Kp, Kd, default_pos 注入节点
    )

    return LaunchDescription([
        rl_node
    ])