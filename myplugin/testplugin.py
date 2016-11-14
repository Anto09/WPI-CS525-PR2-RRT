#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/myplugin')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    RRT = RaveCreateModule(env,'RRT')
    print RRT.SendCommand('help')
finally:
    RaveDestroy()
