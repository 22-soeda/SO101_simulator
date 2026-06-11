from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

setup(
    name="so101_ik_cpp",
    ext_modules=[
        Pybind11Extension(
            "so101_ik_cpp",
            ["so101_ik_cpp.cpp"],
            cxx_std=17,
            extra_compile_args=["/utf-8"],
        ),
    ],
    cmdclass={"build_ext": build_ext},
)
