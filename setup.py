from setuptools import setup, find_packages
import os

# README dosyasını oku
def read_readme():
    readme_path = os.path.join(os.path.dirname(__file__), 'README.md')
    if os.path.exists(readme_path):
        with open(readme_path, 'r', encoding='utf-8') as f:
            return f.read()
    return "FiratROVNet - ROV Simülasyon ve GNC Sistemi"

# Version bilgisini __init__.py'den al
def get_version():
    init_path = os.path.join(os.path.dirname(__file__), 'FiratROVNet', '__init__.py')
    if os.path.exists(init_path):
        with open(init_path, 'r', encoding='utf-8') as f:
            for line in f:
                if line.startswith('__version__'):
                    return line.split('=')[1].strip().strip('"').strip("'")
    return "1.7.4"

setup(
    name="FiratROVNet",
    version=get_version(),
    author="Ömer Faruk Çelik",
    author_email="omerfarukcelik@firat.edu.tr",
    description="ROV Simülasyon ve GNC (Guidance, Navigation, Control) Sistemi",
    long_description=read_readme(),
    long_description_content_type="text/markdown",
    url="https://github.com/FiratROVNet/FiratROVNet",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "License :: OSI Approved :: MIT License",
    ],
    python_requires=">=3.8",
    install_requires=[
        "numpy>=1.20.0",
        "ursina>=5.0.0",
    ],
    extras_require={
        "gat": [
            "torch>=1.10.0",
            "torch-geometric>=2.0.0",
        ],
        "full": [
            "torch>=1.10.0",
            "torch-geometric>=2.0.0",
        ],
    },
    include_package_data=True,
    zip_safe=False,
)





from setuptools import setup, find_packages
import os

# README dosyasını oku
def read_readme():
    readme_path = os.path.join(os.path.dirname(__file__), 'README.md')
    if os.path.exists(readme_path):
        with open(readme_path, 'r', encoding='utf-8') as f:
            return f.read()
    return "FiratROVNet - ROV Simülasyon ve GNC Sistemi"

# Version bilgisini __init__.py'den al
def get_version():
    init_path = os.path.join(os.path.dirname(__file__), 'FiratROVNet', '__init__.py')
    if os.path.exists(init_path):
        with open(init_path, 'r', encoding='utf-8') as f:
            for line in f:
                if line.startswith('__version__'):
                    return line.split('=')[1].strip().strip('"').strip("'")
    return "1.7.4"

setup(
    name="FiratROVNet",
    version=get_version(),
    author="Ömer Faruk Çelik",
    author_email="omerfarukcelik@firat.edu.tr",
    description="ROV Simülasyon ve GNC (Guidance, Navigation, Control) Sistemi",
    long_description=read_readme(),
    long_description_content_type="text/markdown",
    url="https://github.com/FiratROVNet/FiratROVNet",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "License :: OSI Approved :: MIT License",
    ],
    python_requires=">=3.8",
    install_requires=[
        "numpy>=1.20.0",
        "ursina>=5.0.0",
    ],
    extras_require={
        "gat": [
            "torch>=1.10.0",
            "torch-geometric>=2.0.0",
        ],
        "full": [
            "torch>=1.10.0",
            "torch-geometric>=2.0.0",
        ],
    },
    include_package_data=True,
    zip_safe=False,
)










