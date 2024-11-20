from setuptools import find_packages
from setuptools import setup

setup(
    name='husky_msgs',
    version='1.0.8',
    packages=find_packages(
        include=('husky_msgs', 'husky_msgs.*')),
)
