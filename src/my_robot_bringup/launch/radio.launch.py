from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    robot_names = ["Giskard","BB8","Daneel","Jander","C3PO"]
    remap_robot_news = ("robot_news", "my_robot_news")
    robot_news_station_nodes = []

    for name in robot_names:
        robot_news_station_nodes.append( Node(
            package="my_py_pkg",
            executable="robot_news_station",
            name= "robot_news_station_"+ name.lower(),
            remappings=[
                remap_robot_news
            ],
            parameters=[
                {"robot_name": name}
            ]
        ))

    smartphone_node = Node(
        package="my_py_pkg",
        executable="smartphone",
        name="my_smartphone_node",
        remappings=[    
            remap_robot_news
        ]
    )

    for node in robot_news_station_nodes:
        ld.add_action(node)
    ld.add_action(smartphone_node)
    return ld