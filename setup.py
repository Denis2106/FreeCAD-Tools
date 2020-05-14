import sys
import os
import FreeCAD

pathList = [
    ['dev', 'Python'],
    ['dev', 'Python', 'FreeCAD', 'qtest'],
    ['dev', 'CAM']
]

srcInitGuiPath = ['dev', 'Python', 'FreeCAD']
srcInitGuiFile = 'InitGui.py'

pathOpt = None

def makePath(stack, fileName=None):
    global pathOpt
    res =  pathOpt.cloudPath + pathOpt.separator.join(stack)
    if fileName: res += pathOpt.separator + fileName
    return res


def log(txt):
    FreeCAD.Console.PrintMessage(txt)


def copyfile(src, dst):
    if pathOpt.separator == '/':
        copyCmd = 'cp'
    else:
        copyCmd = 'copy'

    os.system('%s %s %s' % (copyCmd, src,dst))


def updateInitGui():
    global pathOpt

    sourceFile = makePath(srcInitGuiPath, srcInitGuiFile)
    try:
        st = os.path.getmtime(sourceFile)
    except:
        st = 0

    try:
        dt = os.path.getmtime(pathOpt.initFile)
    except:
        dt = 0

    if st > dt:
        copyfile(sourceFile, pathOpt.initFile)
        log('setup.py: InitGui.py updated \n')

    elif dt > st:
        copyfile(pathOpt.initFile, sourceFile)
        log('setup.py: template of InitGui.py updated \n')

    else:
        log('setup.py: InitGui.py is actual \n')


def setup(initPathOpt):
    global pathOpt
    pathOpt = initPathOpt

    for path in pathList:
        pathStr = makePath(path)
        sys.path.append(pathStr)
        log('setup.py: Added path %s \n' % pathStr)

    updateInitGui()

    import qtest
    qtest.shortcut()
    log('qtest.shortcut() executed \n')
