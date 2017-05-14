#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/orplugin')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    rrtPlugin = RaveCreateModule(env,'rrtPlugin')
    print rrtPlugin.SendCommand('help')
finally:
    RaveDestroy()
