#!/usr/bin/env python3
"""Setup script for Celica Suspension Analysis System."""

from setuptools import setup, find_packages
from pathlib import Path

# Read README for long description
readme_path = Path(__file__).parent / "README.md"
long_description = readme_path.read_text(encoding="utf-8") if readme_path.exists() else ""

setup(
    name="celica-suspension-analysis",
    version="1.0.0",
    author="Celica Suspension Project",
    description="Suspension data acquisition and analysis system for Toyota Celica (5th gen)",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/zmohammed5/celica_suspension_analysis",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Topic :: Scientific/Engineering",
    ],
    python_requires=">=3.8",
    install_requires=[
        "numpy>=1.21.0",
        "scipy>=1.7.0",
        "pandas>=1.3.0",
        "matplotlib>=3.4.0",
        "plotly>=5.3.0",
        "flask>=2.0.0",
        "flask-socketio>=5.1.0",
        "pyyaml>=5.4.0",
        "python-dateutil>=2.8.0",
        "filterpy>=1.4.0",
        "pynmea2>=1.18.0",
    ],
    extras_require={
        "hardware": [
            "smbus2>=0.4.0",
            "RPi.GPIO>=0.7.0",
            "pyserial>=3.5",
            "obd>=0.7.1",
        ],
        "dev": [
            "pytest>=7.0.0",
            "pytest-cov>=3.0.0",
            "flake8>=4.0.0",
            "black>=22.0.0",
            "mypy>=0.930",
        ],
    },
    entry_points={
        "console_scripts": [
            "celica-daq=src.main:main",
        ],
    },
    include_package_data=True,
    package_data={
        "": ["*.json", "*.yaml", "*.html", "*.css", "*.js"],
    },
)
