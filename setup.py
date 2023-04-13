from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

d = generate_distutils_setup(
    packages=['openai_ros'],
    package_dir={'': 'src'}
)

setup(**d)
