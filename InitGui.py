import os
import FreeCAD

class PathOpt:
    cloudPath = None
    separator = None
    initFile = None
    modulePath = None

pathOpt = PathOpt()

if os.environ.get('HOME', None) == '/Users/denis':
    # for MacOS
    pathOpt.cloudPath = '/Users/denis/SkyDrive/'
    pathOpt.separator = '/'
    pathOpt.initFile = '/Users/denis/Library/Preferences/FreeCAD/Mod/DK/InitGui.py'

elif os.environ.get('HOMEPATH', None) == '\\Users\\dk274':
    # for MS Surface
    pathOpt.cloudPath = 'C:\\Users\\dk274\\OneDrive\\'
    pathOpt.separator = '\\'
    pathOpt.initFile = 'c:\\Users\\dk274\\AppData\\Roaming\\FreeCAD\\Mod\\DK\InitGui.py'

else:
    FreeCAD.Console.PrintError('Unknown system \n')
    FreeCAD.Console.PrintError('HOME=%s \n' % os.environ.get('HOME',''))
    FreeCAD.Console.PrintError('HOMEPATH=%s \n' % os.environ.get('HOMEPATH',''))

pathOpt.modulePath = ['Dev', 'Python', 'FreeCAD']

import sys

pathStr = pathOpt.cloudPath + pathOpt.separator.join(pathOpt.modulePath)
sys.path.append(pathStr)
FreeCAD.Console.PrintMessage('InitGui.py: Added path %s \n' % pathStr)

import setup
setup.setup(pathOpt)
