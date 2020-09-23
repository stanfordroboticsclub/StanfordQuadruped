from setuptools import setup

setup(
    name="stanford_quad",
    version="1.0",
    install_requires=["tqdm", "numpy", "gym"],
    extras_require={
        "sim": ["matplotlib", "pybullet", "gym", "opencv-python", "torchvision"],
        "robot": [
            "transforms3d",
            "UDPComms @ git+https://github.com/stanfordroboticsclub/UDPComms@master#egg=UDPComms",
        ],
    },
)
