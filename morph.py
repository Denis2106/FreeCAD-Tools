'''
Изменяет геометрию объектов FreeCAD
- размеры Box по выбранной плоскости и длине вектора нормали
- размеры Box по выбранной грани и изменению ее длины
- длину LineSegment (от новых координат вершины)
'''
import Part
import FreeCAD as App
import FreeCADGui as Gui


def log(txt):
    App.Console.PrintMessage(txt + '\n')


def morphSelection(val=None, dval=0):
    sel = Gui.Selection.getSelectionEx()[0]

    if sel.Object.TypeId == 'Part::Box':
        morphBox(sel.Object, sel.SubObjects[0], val, dval)


def morphBox(object, subObject, val=None, dval=0):
    correctBase = False

    # Определяем тип изменяемого элемента
    if type(subObject) == Part.Edge:
        direct = subObject.Curve.Direction

    elif type(subObject) == Part.Face:
        direct = subObject.normalAt(0,0)

        v = subObject.CenterOfMass - object.Shape.CenterOfMass
        if v.x + v.y + v.z < 0: correctBase = True

    else:
        raise Exception('morphBox: Некорректный тип элемента %s' + str(type(subObject)))

    nx = object.Placement.Rotation.multVec( App.Vector(1,0,0) )
    ny = object.Placement.Rotation.multVec( App.Vector(0,1,0) )
    nz = object.Placement.Rotation.multVec( App.Vector(0,0,1) )

    if abs(direct * nx) > 0.9:
        propName = 'Length'
        axis = 'x'
    elif abs(direct * ny) > 0.9:
        propName = 'Width'
        axis = 'y'
    elif abs(direct * nz) > 0.9:
        propName = 'Height'
        axis = 'z'
    else:
        log('Грань не является внешней (%s, %s, %s)' % (nx, ny, nz))

    if not val: val = getattr(object, propName).Value

    delta = val + dval - getattr(object, propName).Value
    setattr(object, propName, val + dval)

    if correctBase:
        log('Position correction %s %.2f' % (axis, -delta))
        old = getattr(object.Placement.Base, axis)
        setattr(object.Placement.Base, axis, old - delta)
