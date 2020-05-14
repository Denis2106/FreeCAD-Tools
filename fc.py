'''
FreeCAD utils
-------------
log(*msgs) - Вывод сообщения на панель FreeCAD / Report view
update() - Обновление Gui после изменения положения объектов

isGroup(obj) - Проверка - является ли объект группой FreeCAD
objectByLabel(name) - Поиск объекта во FreeCAD по label
getCenterPoint(label) - Находит центральную точку объекта по метке объекта

matrix(matrix) - Создает матрицу типа App.Matrix из матрицы l3d.Matrix3D
transferPlacement(matrix, placement=None) - Создание App.Placement через матричное преобразование placement

findWidget(path) - Находит виджет по пути определенному в Widget inspector
setComboStyle(style=None, font=None) - Устанавливает стиль для FreeCAD/Combo view
'''

import FreeCAD as App
import FreeCADGui as Gui

# Вывод сообщения на панель FreeCAD / Report view
def log(*msgs):
    for msg in msgs:
        App.Console.PrintLog(msg + '\n')

# Обновление Gui после изменения положения объектов
def update():
    Gui.updateGui()

# Проверка - является ли объект группой FreeCAD
def isGroup(obj):
    return (obj.__class__.__name__ == 'DocumentObjectGroup')

# Поиск объекта во FreeCAD по label
def objectByLabel(name):
    if name == None:
        raise Exception('Поиск объекта None невозможен')

    objects = App.ActiveDocument.getObjectsByLabel(name)
    if not objects or len(objects)==0:
        raise Exception('Не найден объект с меткой %s' % name)
    return objects[0]

# Находит центральную точку объекта по метке объекта
def getCenterPoint(label):
    obj = FC_objectFind(label)
    return l3d.Vector3D(obj.Shape.CenterOfMass)

# Создает матрицу типа App.Matrix из матрицы l3d.Matrix3D
def matrix(matrix):
    import_matrix = matrix.FreeCADmatrix()
    res = App.Matrix(*import_matrix)
    return res

# Преобразует координаты фигуры в соответствии с матрицей - СТАРАЯ РЕАЛИЗАЦИЯ
def transform(objId, matrix, placement=None):
    #   matrix - матрица типа App.Matrix или l3d.Matrix3D
    #   placement - исходное положение фигуры
    if isinstance(objId, Part.Shape):
        obj = None
        shape = objId

    elif isinstance(objId, str):
        obj = FC_objectFind(objId)
        shape = obj.Shape

    shape = shape.copy()

    # Преобразуем матрицу к формату App.Matrix
    if isinstance(matrix, l3d.Matrix3D):
        matrix = matrix(matrix)

    if placement == None:
        placement = shape.Placement

    # Преобразование разделено на движение и поворот, что бы
    # поворот обозначать через квантирионы, т.к. transferShape через матрицы
    # приводит к защелкиваниям
    shape.Placement.Base = matrix.multVec(placement.Base)

    r = App.Rotation()
    r.Q = l3d.M2Q( l3d.Matrix3D(matrix) )
    shape.Placement.Rotation = r.multiply(placement.Rotation)

    # Если передали объект или его метку, обновляем
    if obj != None:
        obj.Shape = shape

    return shape

# Создание App.Placement через матричное преобразование placement
def transferPlacement(matrix, placement=None):
    if placement == None:
        base0 = l3d.Vector3D(0,0,0)
        rot0 = App.Rotation()
    else:
        base0 = l3d.Vector3D(placement.Base)
        rot0 = placement.Rotation

    res = App.Placement()
    base = base0 * matrix
    res.Base = App.Vector(base.x, base.y, base.z)

    r = App.Rotation()
    r.Q = l3d.M2Q( matrix )
    res.Rotation = r.multiply(rot0)

    return res

# Находит виджет по пути определенному в Widget inspector
def findWidget(path):
    nextWidget = widget = Gui.getMainWindow()
    for item in path:
        if type(item) == int:
            nextWidget = widget.children()[item]
        elif 'objectName' in item:
            for c in widget.children():
                if c.objectName() == item['objectName']:
                    nextWidget = c
        elif 'windowTitle' in item:
            for c in widget.children():
                if getattr(c, 'windowTitle', None) and c.windowTitle() == item['windowTitle']:
                    nextWidget = c
        else:
            raise Exception('Неизвестный тип перехода ' + item)

        if nextWidget == widget:
            raise Exception('Ошибка поиска виджета. Путь=%s шаг %s' % (str(path), str(item)))

        widget = nextWidget

    return nextWidget

# Устанавливает стиль для FreeCAD/Combo view
def setComboStyle(style=None, font=None):
    if not style and font:
        style = 'QTreeWidget, QTreeView {font:%dpx}' % font

    treeWidget = findWidget([ {"windowTitle": "Combo View"}, {"windowTitle": "CombiView"}, {"objectName": "combiTab"}, {"objectName": "qt_tabwidget_stackedwidget"}, 2, 1, 0 ])
    treeWidget.setStyleSheet(style)

    dataWidget = findWidget([ {"windowTitle": "Combo View"}, {"windowTitle": "CombiView"}, {"objectName": "combiTab"}, {"objectName": "qt_tabwidget_stackedwidget"}, 2, 0, {"objectName": "propertyTab"}, {"objectName": "qt_tabwidget_stackedwidget"}, 1 ])
    dataWidget.setStyleSheet(style)

    viewWidget = findWidget([ {"windowTitle": "Combo View"}, {"windowTitle": "CombiView"}, {"objectName": "combiTab"}, {"objectName": "qt_tabwidget_stackedwidget"}, 2, 0, {"objectName": "propertyTab"}, {"objectName": "qt_tabwidget_stackedwidget"}, 2 ])
    viewWidget.setStyleSheet(style)
