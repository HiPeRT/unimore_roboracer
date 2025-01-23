import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # map_name_arg = DeclareLaunchArgument(
    #    "map_name", default_value=TextSubstitution(text="mille_conf3")
    # )

    accelerator_name = "rmds"

    var = os.popen('xlnx-config -x listapps').read()
    var = [v.replace("\n", "") for v in var.split(" ") if v]

    var = [var[0+i*5:5+i*5] for i in range(0, len(var)//5)][1:]

    for v in var:
        if v[0] == accelerator_name and v[-1] == "-1":
            print("loading " + accelerator_name)
            os.system("xlnx-config -x loadapp " + accelerator_name)
        elif v[0] == accelerator_name:
            print(accelerator_name + " already loaded")

    return LaunchDescription(
        [
            # map_name_arg,
            Node(
                package="particle_filter",
                executable="particle_filter_node",
                name="particle_filter",
                parameters=[
                    {
                        "conf_path": os.path.join(
                            get_package_share_directory("particle_filter"),
                            "conf",
                            "particle.yaml",
                        ),
                        "ekf_conf_path": os.path.join(
                            get_package_share_directory("particle_filter"),
                            "conf",
                            "ekf.yaml",
                        ),
                        "numRaysDs": "60"),
                    }
                ],
                remappings=[
                    ("/initialpose", "/initialpose"),
                    ("/map", "/map"),
                    ("/odom", "/odom"),
                    ("/scan", "/scan"),
                    ("/thundershot/pf/position", "/thundershot/pf/position"),
                    ("/thundershot/pf/particles", "/thundershot/pf/particles"),
                ],
            )
        ]
    )
