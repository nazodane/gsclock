# Copyright (C) 2024 Toshimitsu Kimura <lovesyao@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause

import os
from setuptools import setup
from glob import glob

def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()


install_requires = [
    "screeninfo",
    "pyserial",
    "pyudev",
    "selenium",
    "wmctrl",
]

from distutils.command.install import install as DistutilsInstall
import os

class MyInstall(DistutilsInstall):
    def run(self):
        os.system("make all")
        DistutilsInstall.run(self)

setup(
    name = "gsclock",
    version = "0.0.1",
    author = "Toshimitsu Kimura",
    author_email = "lovesyao@gmail.com",
    description = ("The Good-shaped Clock"),
    license = "BSD-3-Clause",
    keywords = "clock analog_clock",
    url = "https://github.com/nazodane/gsclock",
    packages=[], #'gsclock_pkg'
    include_package_data = True,
    long_description=read('README.md'),
    python_requires=">=3.10.0",
    install_requires=install_requires,
    scripts=["gsclock"],
    data_files=[
        ('share/gsclock', ["gsclock.html", "avr-gcc-arduino.specs", "gsclock_arduino_uno_sketch.png", "gsclock_arduino.ino", "gsclock_arduino.hex"]),
        ('share/gsclock/styles', glob("styles/*"))
    ],
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Topic :: Desktop Environment",
        "License :: OSI Approved :: BSD License",
        "Programming Language :: Python :: 3.10",
        "Operating System :: POSIX :: Linux",
        "Intended Audience :: End Users/Desktop",
    ],
    cmdclass={'install': MyInstall},
)
