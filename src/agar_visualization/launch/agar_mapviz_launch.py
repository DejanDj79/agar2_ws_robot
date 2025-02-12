import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
 
def generate_launch_description():
    
    mapviz_config = os.path.join(
      get_package_share_directory('agar_visualization'),
      'config',
      'mapviz_config.mvc'
    )

    mapviz_node = Node(
        package="mapviz",
        executable="mapviz",
        name="mapviz",
        parameters = [
            {"config": mapviz_config}
        ]
    )

    swri_transform_node = Node(
        package="swri_transform_util",
        executable="initialize_origin.py",
        name="initialize_origin",
        parameters = [
            {"local_xy_frame" : "map"},
            {"local_xy_origin" : "auto"},
            {"local_xy_navsatfix_topic" : "/gps/fix"},
        ]
    )


    ld = LaunchDescription()
    ld.add_action(mapviz_node)
    ld.add_action(swri_transform_node)

    return ld