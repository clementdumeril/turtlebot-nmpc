from glob import glob

from setuptools import setup

package_name = "tb_nmpc_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*.wbt")),
        ("share/" + package_name + "/resource", glob("resource/*.urdf") + glob("resource/*.yml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Clement Dumeril",
    maintainer_email="clementdumeril@gmail.com",
    description="Webots world and scenarios for tb_nmpc",
    license="MIT",
)
