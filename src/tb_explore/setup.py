from glob import glob

from setuptools import setup

package_name = "tb_explore"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Clement Dumeril",
    maintainer_email="clementdumeril@gmail.com",
    description="Frontier exploration with the NMPC and posed photo capture",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "explorer = tb_explore.explorer:main",
            "auto_capture = tb_explore.auto_capture:main",
            "simple_mapper = tb_explore.simple_mapper:main",
        ],
    },
)
