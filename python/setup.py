import os
import warnings
from glob import glob

from pybind11.setup_helpers import ParallelCompile, naive_recompile
from pybind11.setup_helpers import Pybind11Extension
from setuptools import setup

# names of the environment variables that define osqp and openrobots include directories
osqp_path_var = 'OSQP_INCLUDE_DIR'
openrobots_path_var = 'OPENROBOTS_INCLUDE_DIR'

__version__ = "5.2.8"
__libraries__ = ['state_representation', 'clproto', 'controllers', 'dynamical_systems', 'robot_model']
__include_dirs__ = ['include']

__install_clproto_module__ = True
__install_controllers_module__ = True
__install_dynamical_systems_module__ = True
__install_robot_model_module__ = True

# check that necessary libraries can be found
try:
    eigen_dir = os.popen(
        'cmake --find-package -DNAME=Eigen3 -DCOMPILER_ID=GNU -DLANGUAGE=C -DMODE=COMPILE').read().strip()
    if eigen_dir.startswith('-I'):
        __include_dirs__.append(eigen_dir.lstrip('-I'))
    else:
        raise Exception('Could not find Eigen3 package!')

    for lib in __libraries__:
        status = os.popen(f'ldconfig -p | grep {lib}').read().strip()
        if len(status) == 0:
            msg = f'Could not find library {lib}!'
            if lib == 'clproto':
                warnings.warn(f'{msg} The clproto module will not be installed.')
                __install_clproto_module__ = False
            elif lib == 'dynamical_systems':
                warnings.warn(f'{msg} The dynamical_systems module will not be installed.')
                __install_dynamical_systems_module__ = False
            elif lib == 'robot_model':
                warnings.warn(f'{msg} The robot_model module will not be installed.')
                __install_robot_model_module__ = False
            elif lib == 'controllers':
                warnings.warn(f'{msg} The controllers module will not be installed.')
                __install_controllers_module__ = False
            else:
                raise Exception(msg)

    if __install_robot_model_module__:
        osqp_path = os.environ[osqp_path_var] if osqp_path_var in os.environ.keys() else '/usr/local/include/osqp'
        __include_dirs__.append(osqp_path)
        openrobots_path = os.environ[
            openrobots_path_var] if openrobots_path_var in os.environ.keys() else '/opt/openrobots/include'
        __include_dirs__.append('/opt/openrobots/include')

    if __install_controllers_module__ and not __install_robot_model_module__:
        warnings.warn(
            'The robot model module is required to build the controllers module! '
            'The controllers module will not be installed with the current settings.')
        __install_controllers_module__ = False

except Exception as e:
    msg = f'Error with control library dependencies: {e.args[0]} Ensure the control libraries are properly installed.'
    warnings.warn(msg)

ParallelCompile('NPY_NUM_BUILD_JOBS', needs_recompile=naive_recompile).install()

ext_modules = [
    Pybind11Extension('state_representation',
                      sorted(glob('source/state_representation/*.cpp') + glob('source/common/*.cpp')),
                      cxx_std=17,
                      include_dirs=__include_dirs__,
                      libraries=['state_representation'],
                      define_macros=[('MODULE_VERSION_INFO', __version__)],
                      )
]

if __install_clproto_module__:
    ext_modules.append(
        Pybind11Extension('clproto',
                          sorted(glob('source/clproto/*.cpp') + glob('source/common/*.cpp')),
                          cxx_std=17,
                          include_dirs=__include_dirs__,
                          libraries=['state_representation', 'clproto'],
                          define_macros=[('MODULE_VERSION_INFO', __version__)],
                          )
    )

if __install_dynamical_systems_module__:
    ext_modules.append(
        Pybind11Extension("dynamical_systems",
                          sorted(glob("source/dynamical_systems/*.cpp") + glob("source/common/*.cpp")),
                          cxx_std=17,
                          include_dirs=__include_dirs__,
                          libraries=['state_representation', 'dynamical_systems'],
                          define_macros=[('MODULE_VERSION_INFO', __version__)],
                          )
    )

if __install_robot_model_module__:
    ext_modules.append(
        Pybind11Extension('robot_model',
                          sorted(glob('source/robot_model/*.cpp')),
                          cxx_std=17,
                          include_dirs=__include_dirs__,
                          libraries=['state_representation', 'robot_model'],
                          define_macros=[('MODULE_VERSION_INFO', __version__)],
                          )
    )

if __install_controllers_module__:
    ext_modules.append(
        Pybind11Extension('controllers',
                          sorted(glob('source/controllers/*.cpp') + glob("source/common/*.cpp")),
                          cxx_std=17,
                          include_dirs=__include_dirs__,
                          libraries=['state_representation', 'controllers', 'robot_model'],
                          define_macros=[('MODULE_VERSION_INFO', __version__)],
                          )
    )

setup(
    name='control-libraries',
    version=__version__,
    author='Enrico Eberhard',
    author_email='enrico.eberhard@epfl.ch',
    url='https://github.com/epfl-lasa/control-libraries',
    description='Python bindings for the C++ control libraries',
    long_description='',
    ext_modules=ext_modules,
    test_suite='tests',
    python_requires='>=3',
    install_requires=[
        'pyquaternion>=0.9.9'
    ],
    license='GNU GPL v3',
    zip_safe=False,
)
