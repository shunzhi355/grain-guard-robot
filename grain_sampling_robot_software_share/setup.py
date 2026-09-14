"""Setup script for grain_sampling_robot_software."""

from setuptools import setup, find_packages

setup(
    name="grain_sampling_robot_software",
    version="0.1.0",
    description="Full-stack software for a grain sampling robot",
    author="Grain Sampling Team",
    author_email="dev@example.com",
    url="https://github.com/example/grain_sampling_robot_software",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    python_requires=">=3.10",
    install_requires=[
        "PySide6",
        "paho-mqtt",
        # ROS1 rospy is provided by the system ROS installation, not PyPI.
        "numpy",
        "opencv-python",
        "pyyaml",
        "pyserial",
    ],
    entry_points={
        "console_scripts": [
            "grain-sampling-ui=grain_sampling_ui.main:main",
            "mock-robot=mock_robot.main:main",
        ],
    },
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3.10",
    ],
)
