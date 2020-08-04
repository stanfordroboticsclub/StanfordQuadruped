from setuptools import setup

setup(
    name="stanford_quad",
    version="1.0",
    install_requires=[
        "tqdm",
        "numpy",
        "matplotlib",
        "pybullet",
        "gym",
        "opencv-python",
        "transforms3d",
        "UDPComms @ git+ssh://git@github.com/stanfordroboticsclub/UDPComms@master#egg=UDPComms",
    ],
)
