from distutils.extension import Extension
from distutils.core import setup

from Cython.Build import cythonize
extensions = cythonize([
    Extension("ublox_msg", ["ublox_msg.pyx"]),
])

setup(
    ext_modules = extensions
)
