from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
      packages=['logic_module'],
      package_dir={'': 'src'}
      )

setup(**d)
