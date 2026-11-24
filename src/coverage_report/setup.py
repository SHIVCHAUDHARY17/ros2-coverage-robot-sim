from setuptools import setup

package_name = "coverage_report"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},  # tells colcon to run the tests with pytest
    zip_safe=True,
    maintainer="Shiv Jayant Chaudhary",
    maintainer_email="shivchaudhary1716j@gmail.com",
    description="Run report (rosbag2 -> HTML/JSON) and live coverage check",
    license="MIT",
    entry_points={
        "console_scripts": [
            "report = coverage_report.report:main",
            "check = coverage_report.check:main",
        ],
    },
)
