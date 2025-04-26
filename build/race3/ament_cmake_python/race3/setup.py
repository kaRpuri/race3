from setuptools import find_packages
from setuptools import setup

setup(
    name='race3',
    version='0.0.0',
    packages=find_packages(
        include=('race3', 'race3.*')),
)
