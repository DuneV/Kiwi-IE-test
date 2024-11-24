from setuptools import setup, find_packages

package_name = "fail_detection_py"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Wilmer",
    maintainer_email="wilmer.garzon@kiwibot.com",
    description="Python version of fail_detection",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fail_detection = fail_detection_py.main:main",  # Asegúrate de apuntar al archivo correcto
        ],
    },
)
