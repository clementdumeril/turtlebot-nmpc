from glob import glob

from setuptools import setup

package_name = "tb_nmpc"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Clement Dumeril",
    maintainer_email="clementdumeril@gmail.com",
    description="acados NMPC with lidar obstacle prediction for TurtleBot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "nmpc_node = tb_nmpc.nmpc_node:main",
            "nav2_follow = tb_nmpc.nav2_follow:main",
        ],
    },
)
