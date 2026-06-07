# setup.py
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['qt_piper_class'],  # 声明子包
    package_dir={'': 'src'},      # 源码路径
)
setup(**setup_args)