from setuptools import setup, find_packages

package_name = "dodo_mujoco_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="your-name",
    maintainer_email="your@email.com",
    description="MuJoCo sim2sim bridge for Dodo robot",
    license="MIT",
    entry_points={
        "console_scripts": [
            "hello_node = dodo_mujoco_bridge.hello_node:main",
        ],
    },
)