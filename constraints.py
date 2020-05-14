'''
Модуль условий
- синхронизирует перемещение объектов внутри группы
- привязывает элемент объекта для

Варианты связи:
- группа FreeCAD
- созданное условие

Условие определяется параметрами
--------------------------------
- имя объекта оказывающего влияние
- зависимый объект или группа
- элемент объекта оказывающего влияние
- элемент зависимого объекта
- тип связи

Типы связей
-----------
object-group - привязаны все объекты группы
element-element - привязана позиция вершины
'''
import math
import FreeCAD as App
from PySide2 import QtCore

import lib3D as l3d
import fc

# Запись в Objects
class ObjectsRec:
    obj = None
    placement = None    # копия положения объекта тип App.Placement
#    group = None        # ссылка на группу типа DocumentObjectGroup


class Constraint:
    def __init__(self, group=None):
        self.Objects = []
        self.isActive = True

        self.group = group
        if group: self.loadGroup()

    def setActive(self, isActive):
        self.isActive = isActive

    # Загрузка Objects из App.ActiveDocument.Objects[].Groups
    def loadGroup(self):
        self.Objects = {}

        for obj in self.group.Group:
            rec = ObjectsRec()
            rec.obj = obj
            rec.placement = obj.Placement.copy()
            self.Objects[obj.Name] = rec

    # Обработка события от AppDocumentObserver
    def processAppDocSignal(self, *args):
        signalType = args[0]    # Тип сигнала из class AppObserver
        obj = args[1]        # Объект который изменился

        if signalType not in ['ObjChanged', 'ObjDeleted']: return False

        if obj == self.group and signalType == 'ObjChanged':
            self.loadGroup()

        elif obj.Name in self.Objects:
            if signalType == 'ObjChanged':
                if self.isActive: self.moveGroupForObject(obj)
                self.savePlacements()

            elif signalType == 'ObjDelete':
                del self.Objects[obj.Name]

            else:
                return False

        else:
            return False

        return True

    # Сохраняет положение объектов в self.Objects
    def savePlacements(self):
        for rec in self.Objects.values():
            self.Objects[rec.obj.Name].placement = rec.obj.Placement.copy()

    # Перемещает все объекты группы в соответствии с перемещение object относительно oldPlacement
    def moveGroupForObject(self, obj):

        newPlacement = obj.Placement
        oldPlacement = self.Objects[obj.Name].placement

        matrix = l3d.Matrix3D() \
                    .moveInc(-oldPlacement.Base)    \
                    .rotateInc(oldPlacement.Rotation.Axis, oldPlacement.Rotation.Angle * 180/math.pi) \
                    .rotateInc(-newPlacement.Rotation.Axis, newPlacement.Rotation.Angle * 180/math.pi) \
                    .moveInc(newPlacement.Base)

        lst = [rec.obj for rec in self.Objects.values() if rec.obj!=obj]
        for o in lst:
            o.Placement = fc.transferPlacement(matrix, o.Placement)


import observer as obs

class ConstraintManager:
    def __init__(self):
        self.constraints = {}
        self._model = None
        self.callback = None

        self.observer = obs.AppObserver(self)
        self.observer.callback = self.processAppDocSignal
        self.observer.connect()
        self.loadGroups()

    def disconnect(self):
        self.observer.disconnect()

    def activityPause(self):
        self.linkActives = {}
        for  name, cons in self.constraints.items():
            self.linkActives[name] = cons.isActive
            cons.isActive = False

    def activityContinue(self):
        for name, isActive in self.linkActives.items():
            self.constraints[name].setActive(isActive)

    def setCallback(self, callback):
        self.callback = callback
        self.callCallback()


    def callCallback(self, *args):
        if self.callback:
            self.callback(*args)


    def updateModel(self):
        if self._model: self._model.sourceChanged()


    def constraintNameForGroup(self, group):
        if not getattr(group,'Name', None):
            raise Exception('Объект %s не имеет Name' % str(group))
        return 'Group.' + group.Name


    def processAppDocSignal(self, *args):
        signalType = args[0]
        obj = args[1]
        if len(args) > 2:
            prop = args[2]
        else:
            prop = ''

        isUpdated = False

        #fc.log('manager.process: %s %s %s' % (signalType, str(obj), prop))

        if fc.isGroup(obj):
            constraintName = self.constraintNameForGroup(obj)

            if signalType == 'ObjDeleted':
                if constraintName in self.constraints:
                    del self.constraints[constraintName]

            elif signalType == 'ObjChanged':
                if constraintName in self.constraints:
                    self.constraints[constraintName].loadGroup()

                elif prop != 'Group':
                    self.addConstraintGroup(obj)

            isUpdated = True

        else:
            if signalType == 'ObjChanged':
                self.observer.disconnect()
                for cons in self.constraints.values():
                    cons.processAppDocSignal(*args)
                self.observer.connect()

                isUpdated = True

        if isUpdated:
            self.updateModel()
            self.callCallback()

        return


    def addConstraintGroup(self, group):
        constraintName = self.constraintNameForGroup(group)
        cons = Constraint(group)
        cons.loadGroup()
        cons.setActive(True)
        self.constraints[constraintName] = cons

        self.updateModel()


    def loadGroups(self):
        if not App.ActiveDocument: return

        groups = [obj for obj in App.ActiveDocument.Objects if fc.isGroup(obj)]

        # Удаляем группы которых уже нет в документе
        lst = [name for name, cons in self.constraints.items() \
                        if name.startswith('Group.') and cons not in groups]
        for name in lst: del self.constraints[name]

        # Добавляе группы которых небыло
        groups = [obj for obj in groups \
                        if self.constraintNameForGroup(obj) not in self.constraints]
        #fc.log('constrints.loadGroup: groups=%s' % str(groups))
        for g in groups:
            self.addConstraintGroup(g)


    def model(self):
        if not self._model:
            self._model = ConsManagerModel(self)
            #self._model = QtCore.QStringListModel(['Line1', 'Line2'])

        return self._model


class ConsManagerModel(QtCore.QAbstractListModel):
    def __init__(self, consMng):
        QtCore.QAbstractListModel.__init__(self)

        self.consMng = consMng
        self.keys = None

        self.loadKeys()


    def rowCount(self, parent=QtCore.QModelIndex):
        return len(self.keys)

    def flags(self, index):
        return QtCore.Qt.ItemIsUserCheckable     \
                | QtCore.Qt.ItemIsEnabled           \
                #| QtCore.Qt.ItemIsEditable            \
                #| QtCore.QAbstractItemModel.flags(index)

    def data(self, index, role):
        value = None

        if index.isValid():
            if role == QtCore.Qt.DisplayRole:
                value = 'GroupLink: %s (%d objects)' % ( \
                        self.getConstraint(index.row()).group.Label,
                        len(self.getConstraint(index.row()).Objects))

            elif role == QtCore.Qt.CheckStateRole:
                value =  QtCore.Qt.Checked if self.getConstraint(index.row()).isActive else QtCore.Qt.Unchecked

        return value


    def setData(self, index, value, role):

        if role == QtCore.Qt.CheckStateRole:
            self.getConstraint(index.row()).setActive( value==QtCore.Qt.Checked )
            self.dataChanged.emit(index, index)
            return QtCore.Qt.Unchecked

            #self.dataChanged.emit(index, index)

            return True
        return False

    # Методы зависящие от объекта
    def getConstraint(self, row):
        return self.consMng.constraints[self.keys[row]]

    def loadKeys(self):
        self.keys = list(self.consMng.constraints.keys())

    def sourceChanged(self):
        self.loadKeys()
        self.dataChanged.emit(None, None)


CM = None

def start():
    global CM
    CM = ConstraintManager()

def stop():
    global CM
    CM.disconnect()
    del CM
