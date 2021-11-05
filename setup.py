# -*- coding: utf-8 -*-
import glob
import os
import shutil

from setuptools import setup
from setuptools.command.build_ext import build_ext

# A CMakeExtension needs a sourcedir instead of a file list.
# The name must be the _single_ output extension from the CMake build.
# If you need multiple extensions, see scikit-build.


class CMakeBuild(build_ext):
    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        # required for auto-detection of auxiliary "native" libs
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep
        os.makedirs(extdir, exist_ok=True)
        cmake_build_dir = os.getenv("cmake_build_dir")
        for f in glob.glob(os.path.join(cmake_build_dir, "**", "*cyy_algorithm*")):
            if not f.endswith(".so"):
                continue
            shutil.copy(f, extdir)


# The information here can also be placed in setup.cfg - better separation of
# logic and declaration, and simpler if you include description/version in a file.
setup(
    name="cyy_algorithm",
    version="0.1",
    author="cyy",
    author_email="cyyever@outlook.com",
    description="Python binding for this C++ lib",
    url="https://github.com/cyyever/naive_cpp_lib",
    cmdclass={"build_ext": CMakeBuild},
    zip_safe=False,
    python_requires=">=3.6",
)
