from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    cylinder_sdf = PathJoinSubstitution(
        [FindPackageShare("gpg_urdf"), "urdf", "cylinder.sdf"]
    )

    cylinders = [
        ("cyl_1", 0.0, 1.0, 0.0),
        ("cyl_2", 2.0, -2.0, 0.0),
        ("cyl_3", 3.0, 0.0, 0.0),
        ("cyl_4", 2.0, 2.0, 0.0),
        ("cyl_5", -2.0, 0.0, 0.0),
        ("cyl_6", -2.0, -2.0, 0.0),
        ("cyl_7", -3.0, 2.0, 0.0),
    ]

    spawns = []

    for name, x, y, z in cylinders:
        spawns.append(
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "ros_gz_sim", "create",
                    "-name", name,
                    "-file", cylinder_sdf,
                    "-x", str(x),
                    "-y", str(y),
                    "-z", str(z)
                ],
                output="screen"
            )
        )

    return LaunchDescription(spawns)
