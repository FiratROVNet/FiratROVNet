from setuptools import Extension, setup

try:
    import numpy as np
except ImportError as exc:
    raise SystemExit("NumPy is required to build native extensions.") from exc


setup(
    name="FiratROVNet-native",
    ext_modules=[
        Extension(
            "FiratROVNet.native.gat_fast",
            sources=["FiratROVNet/native/gat_fast.c"],
            include_dirs=[np.get_include()],
            extra_compile_args=["-O3"],
        )
    ],
)

