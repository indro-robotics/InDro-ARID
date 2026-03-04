from setuptools import setup

package_name = "realsense_cv_pipe"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/realsense_cv_pipe.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Calvin-InDro",
    maintainer_email="calvin.rubens@indrorobotics.com",
    description="Composable rectification + extra node for Realsense.",
    license="Apache-2.0",
    entry_points={},
)