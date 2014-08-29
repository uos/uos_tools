from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()

d['packages'] = ['simple_approximate_time_synchronizer']
d['package_dir'] = {'':'src'}
d['install_requires'] = []

setup(**d)
