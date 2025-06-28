__version__ = "0.0.1"

# Let users know if they're missing any of our hard dependencies
hard_dependencies = ("numpy", "scipy", "lxml")
missing_dependencies = []

for dependency in hard_dependencies:
    try:
        __import__(dependency)
    except ImportError as e:
        missing_dependencies.append("{0}: {1}".format(dependency, str(e)))

if missing_dependencies:
    raise ImportError(
        "Unable to import required dependencies:\n" +
        "\n".join(missing_dependencies)
    )
del hard_dependencies, dependency, missing_dependencies

from . import cython_ubsplclib
from .cython_ubsplclib import *

__all__=[]
__all__.extend(cython_ubsplclib.__all__)