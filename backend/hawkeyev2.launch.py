from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    package_share = get_package_share_directory('hawkeyev2')

    yolo_model = os.path.join(
        package_share,
        'models',
        'yolov8n.onnx'
    )

    pose_model = os.path.join(
        package_share,
        'models',
        'blazepose.onnx'
    )

    hand_model = os.path.join(
        package_share,
        'models',
        'blazehand.onnx'
    )

    return LaunchDescription([
        Node(
            package='hawkeyev2',
            executable='hawkeyev2',
            name='yolo_vision_node',
            output='screen',
            parameters=[{
                'yolo_model_path': yolo_model,
                'pose_model_path': pose_model,
                'hand_model_path': hand_model,
            }],
        )
    ])