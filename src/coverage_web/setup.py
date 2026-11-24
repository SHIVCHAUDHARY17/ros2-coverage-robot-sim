from setuptools import setup

package_name = "coverage_web"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_data={package_name: ["static/index.html"]},  # served by app.py
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},  # tells colcon to run the tests with pytest
    zip_safe=False,  # index.html is read from the installed package directory
    maintainer="Shiv Jayant Chaudhary",
    maintainer_email="shivchaudhary1716j@gmail.com",
    description="Operator web UI (FastAPI + WebSocket) for the simulated coverage robot",
    license="MIT",
    entry_points={"console_scripts": ["server = coverage_web.server:main"]},
)
