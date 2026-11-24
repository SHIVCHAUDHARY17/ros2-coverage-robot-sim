from glob import glob

from setuptools import setup

package_name = "coverage_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/data", glob("data/*.jpg")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},  # tells colcon to run the tests with pytest
    zip_safe=True,
    maintainer="Shiv Jayant Chaudhary",
    maintainer_email="shivchaudhary1716j@gmail.com",
    description="Classical-CV perception: spray-painted marking -> target grid",
    license="MIT",
    entry_points={"console_scripts": ["perception_node = coverage_perception.perception_node:main"]},
)
