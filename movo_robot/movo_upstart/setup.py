from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(
    packages=['movo_upstart'],
    package_dir={'': 'src'}
))
