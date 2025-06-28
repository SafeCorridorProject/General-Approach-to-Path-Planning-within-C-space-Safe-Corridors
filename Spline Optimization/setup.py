# -*- coding: utf-8 -*-
from setuptools import setup, Extension
from numpy import get_include
import os
import re

_VERSION = None
with open(os.path.join('Bspl_Lib', '__init__.py'), 'r') as f:
    _VERSION = re.search(r'__version__\s*=\s*"([^"]+)"', f.read()).group(1)

assert _VERSION

USE_CYTHON = True
ext = ".pyx" if USE_CYTHON else ".c"

EXT_MODULES = [
        Extension("Bspl_Lib.cython_ubsplclib",
                  sources = ["Bspl_Lib/cython_ubsplclib"+ext,
                             "Bspl_Lib/ubsplclib.c"],
                  include_dirs = [".", get_include()])
        ]

if USE_CYTHON:
    from Cython.Build import cythonize
    EXT_MODULES = cythonize(
        EXT_MODULES,
        compiler_directives={"language_level": "3"}
        )

setup(version = _VERSION,
      packages = ["Bspl_Lib"],
      package_data = {"Bspl_Lib": ["*.pxd"]},
      ext_modules = EXT_MODULES,
      zip_safe = False,
      )