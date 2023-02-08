#!/usr/bin/env python

# DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from distutils import util

PKG_NAME = "group6_rwa4"
package_path = util.convert_path('src/' + PKG_NAME)
robots_path = util.convert_path(package_path + '/robots')
scheduler_path = util.convert_path(package_path + '/scheduler')
tasks_path = util.convert_path(package_path + '/tasks')
utils_path = util.convert_path(package_path + '/utils')

setup(
    version="0.0.0",
    package_dir={
        PKG_NAME: package_path,
        'robots': robots_path,
        'scheduler': scheduler_path,
        'tasks': tasks_path,
        'utils': utils_path},
    packages=[PKG_NAME, 'robots', 'scheduler', 'tasks', 'utils'],
    install_requires=['numpy']
)
