"""setup.py for Pupper V2 Controller.

Install for development:

  pip intall -e .
"""

from setuptools import find_packages
from setuptools import setup

setup(
    name="pupper_controller",
    version="0.0.1",
    description=("Controller for Pupper V2"),
    author="Pupper Authors",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    packages=find_packages(),
    install_requires=[
        "gym",
        "numpy",
        "pyserial",
        "hid",
        "transforms3d",
        "scipy",
        "quadprog",
        "pybullet",
        "arspb",
        "absl-py",
        "ray",
        "matplotlib",
        "pupper_hardware_interface @ git+https://github.com/stanfordroboticsclub/Pupper-V2-Interface.git",
    ],
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
    ],
    keywords="Pupper Simulation quadruped robot reinforcement learning rigidbody physics",
)
