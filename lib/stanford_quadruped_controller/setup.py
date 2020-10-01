import setuptools

setuptools.setup(
    name="stanford_quadruped_controller",
    version="1.0",
    author="Nathan Kau",
    author_email="nathankau@gmail.com",
    description="Core functions for controlling quadruped robots",
    packages=["stanford_quadruped_controller"],
    install_requires=[
        "numpy",
        "pyyaml",
        "transforms3d",
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
)
