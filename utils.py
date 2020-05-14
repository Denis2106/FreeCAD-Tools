# Copyright © 2020 Denis A. Kim. All rights reserved.
'''

slicePart(obj, planes[]) - разрезает объект на части и возвращает вырезанный объект
! plane.section(Object.Shape) - дает контур реза
cutPolygon(obj, plane, start, end) - возвращает полигон резка объекта плоскостью

createCutPie(shape, segment=45, angle=0, pos=0, height=2, axis=None, radius=None)
createCutBox(shape, width=20, angleX=0, pos=0, height=2) - TODO
getCutWire(shape) - TODO возвращает массив точек контура реза

createAlonePlane(shape, angleZ = 0, size=20)
createCrossPlane(shape, pos = 0, size=20)
moveL(shape, dir, len) - перемещение shape в ЛСК
rotateL(shape, axisL, angle, pointL=None): - поворот shape вокруг оси в ЛСК
#  Obj.Placement.Rotation.Angle = angle in rad
getL2Gmatrix(shape) - DEPRECATED возвращает матрицу преобразования координат L2G
calcL2G(matrix, vector) - преобразование координат L2G по матрице L2G
calcG2L(matrix, vector) - преобразование координат G2L по матрице L2G

checkObjBetweenPlanes(mainShape, shape, planes[]) - проверяет что фигура между плоскостями
findMainAxis(shape) - определение основной оси фигуры
planeNormal(plane) - вектор нормали к плоскости
distVertexToPlanes(vertex, planes[]) - массив расстояний от vertex до planes
shapeSize(shape) - возвразает мин. и макс. координаты фигуры в ЛСК
getSelectionPlacement(index=0) - возвращает глобальный Placement для выбранного объекта
getAxes(shape) - возвращает вектора локальных осей объекта

showSection(shape, tools, name=None) - показывает пересечение shape и tools
showShapes(shapes) - показывает фигуры из массива
showAxes(shape, size=10) - рисует локальные оси объекта длины size
showVector(point, vector, name=None, color=None) - рисует вектор из точки
show(obj, name=None, size=5) - рисует obj (Vector, Vertex)
showLBoundBox(obj) - рисует BoudBox объекта в координатах объекта
showEdge(e, Name)
showEdges(es)

_str(obj, short=False, origin=None) - строка описания объекта с трансляцией координат
list(obj, origin=None) - печатает список объектов с трансляцией координат
listAxes(shape) - выводи на консоль список осей
listVertexes(shape) - выводит на консоль список вершин из shape.Vertexes
info(a, all=False, Type='', Name='') - выдает в консоль свойства и методы объекта
'''
#from importlib import reload  # функция перезагрузки модуля

import Part, math
from FreeCAD import Vector
from FreeCAD import Rotation
import FreeCAD as App
import FreeCADGui as Gui

def ref(label):
    res = App.ActiveDocument.getObjectsByLabel(label)
    if len(res):
        return res[0]
    else:
        return None

def angle(Ref1, Ref2):
    l1 = App.ActiveDocument.getObjectsByLabel(Ref1)[0]
    v1 = l1.Points[1] - l1.Points[0]

    if type(Ref2) == str:
        l2 = App.ActiveDocument.getObjectsByLabel(Ref2)[0]
        v2 = l2.Points[1] - l2.Points[0]

    elif type(Ref2) == tuple:
        v2 = Vector(Ref2)

    elif type(Ref2) == Vector:
        v2 = Ref2

    print('v1_v2_angle=%.1f' % (v1.getAngle(v2) * 180 / math.pi  ))
    print('v1_x_angle=%.1f' % (v1.getAngle(Vector(1,0,0)) * 180 / math.pi  ))
    print('v1_y_angle=%.1f' % (v1.getAngle(Vector(0,1,0)) * 180 / math.pi  ))
    print('v1_z_angle=%.1f' % (v1.getAngle(Vector(0,0,1)) * 180 / math.pi  ))

    #return v1.getAngle(v2) * 180 / math.pi


#===============================================================
# Преобразование объектов
#===============================================================
def slicePart(shape, tools, showSliced=True):
    ''' Вырезает из большого объекта маленький
        последовательно применяя фигуры из Tools

        Пример:
        ap1 = createAlonePlane(obj); Part.show(ap1,'ap1')
        ap2 = createAlonePlane(obj, 20); Part.show(ap2,'ap2')
        cp1 = createAlonePlane(obj); Part.show(cp1,'cp1')
        cp2 = createAlonePlane(obj); Part.show(cp2,'cp2')

        s=u.slicePart(Obj, [ap1, ap2, cp1, cp2])

        f = App.ActiveDocument.Objects[-1]
        Obj.Shape.common(f.Shape).Volume
    '''
    import BOPTools.SplitFeatures

    sliced = BOPTools.SplitAPI.slice(shape, tools, "Split")

    if showSliced:
        Part.show(sliced)

    for s in sliced.Solids:
        if checkObjBetweenPlanes(shape, s, tools):
            s1 = s.transformGeometry(shape.Matrix.inverse())
            s1.Placement = shape.Placement
            Part.show(s1)
            return s1
#---------------------------------------------------------------
def cutPolygon(obj, plane, start, end):
    ''' Расчитывает полигон резка объекта (Obj) плоскостью (plane)
        возвращает объект типа TopoShape
    '''
    cut = plane.section(obj.Shape)
    edge = cut.Edges[0]
    if not start:
        start = edge.FirstParameter
    if not end:
        end = edge.LastParameter
    cutEdge = edge.discretize(Deflection=0.1, First=start, Last=end)
    cutPoly = Part.makePolygon(cutEdge)
    return cutPoly
#===============================================================
# Инструменты для выделения фрагмента реза
#===============================================================
def findMaxLength(array):
    '''Ищет элемент с максимальным значением атрибута .Length
    '''
    item = None
    for i in array:
        if item==None or i.Length > item.Length: item = i
    return item
#------------------------------------------------------------------------------
def cutEdge(edge, plane1, plane2, tolerance = 0.001):
    edges = edge.cut([plane1, plane2]).Edges

    for k in range(2):
        i = 0
        while i < len(edges):
            p = edges[i].Vertexes[1]
            if min( p.distToShape(plane1)[0], p.distToShape(plane2)[0]) > 0.01:
                # Если конец грани не на плоскостях деления
                # ищем грань которая может быть пристыкована к этой и сращиваем
                j = 0
                while j < len(edges):
                    if i!=j and p.distToShape( edges[j].Vertexes[0] )[0] < 0.01:
                        # нашли грань что бы пристыковать
                        edges[i] = Part.Wire([edges[i], edges[j]])
                        edges.pop(j)
                    else:
                        j += 1
            i += 1
        if len(edges) <=2: break

    if edges[0].Length < edges[1].Length:
        return Part.Wire(edges[0])
    else:
        return Part.Wire(edges[1])
#------------------------------------------------------------------------------
def createFragment(shape, x=0, a=0, width=20, height=20):
    cp1 = createCrossPlane(shape, x)
    cp2 = createCrossPlane(shape, x+width)
    ap1 = createAlonePlane(shape, a)
    ap2 = createAlonePlane(shape, a+height)

    # Поиск грани с максимальной длиной для сечений CrossPlane
    ec1 = findMaxLength(shape.section(cp1).Edges)
    ec2 = findMaxLength(shape.section(cp2).Edges)

    # Поиск граней AlonePlane пересекающихся с выбранными гранями createCrossPlane
    r = []
    for e in shape.section(ap1).Edges:
        if e.distToShape(ec1)[0] < 0.01 or e.distToShape(ec2)[0] < 0.01:
            r.append(e)

    if len(r)==1: ea1 = r[0]
    elif len(r)==2: ea1 = Part.Wire(r)
    else: print('Много подходящих граней', r)

    # Поиск граней AlonePlane пересекающихся с выбранными гранями createCrossPlane
    r = []
    for e in shape.section(ap2).Edges:
        if e.distToShape(ec1)[0] < 0.01 or e.distToShape(ec2)[0] < 0.01:
            r.append(e)

    if len(r)==1: ea2 = r[0]
    elif len(r)==2: ea2 = Part.Wire(r)
    else: print('Много подходящих граней', r)

    # Определяем точки на пересечении граней
    p1 = ec1.distToShape(ea1)[1][0][0]
    p2 = ec1.distToShape(ea2)[1][0][0]
    p3 = ec2.distToShape(ea2)[1][0][0]
    p4 = ec2.distToShape(ea1)[1][0][0]

    # Обрезаем грани по точкам пересечения с перпендикулярными плоскостями
    ec1 = cutEdge(ec1, ap1, ap2)
    ec2 = cutEdge(ec2, ap1, ap2)
    ea1 = cutEdge(ea1, cp1, cp2)
    ea2 = cutEdge(ea2, cp1, cp2)

    return (p1,p2,p3,p4), (ea1,ec1,ea2,ec2)


#===============================================================
# Создание примитивов
# Алгоритм создания объекта:
# - создаем объект с нужными размерами в начале ГСК
# - поворачиваем объект в нужном направлении через Placement.Rotation
# - перемещаем объект в начальную точку ЛСК через Placement.base
# - поворачиваем в локальной системе координат через rotateL
# - передвигаем в локальной системе координат через moveL
#===============================================================
def createTopCone(face, placement, pos=30, r_angle=0, segment=45, height=2):
    '''Создает конус рассекающий сферическую поверхность
       face - рассекаемая поверхность
       placement - базис поверхности в ГСК
       pos - удаление контура от центра рассекаемой поверхности
       r_angle - начальный угол в градусах
       segment - угловой размер в градусах
       height - толщина конуса
    '''
    radius = face.Surface.Radius
    cone_height = math.sqrt(radius**2 - pos**2)

    cone = Part.makeCone(0, pos, cone_height, Vector(0,0,0), Vector(0,0,1), segment)

    ax,ay,az = getAxes(placement)
    cone.Placement.Rotation = Rotation(Vector(0,0,1), ax)
    sax,say,saz = getAxes(cone)
    #rotateL(s, (1,0,0), 180)
    cone.Placement.Base = placement.Base

    return cone
#---------------------------------------------------------------
def createCutBox(shape, width=20, angle=0, pos=0, height=2, radius=None):
    '''Создает пластину которая режет shape вдоль главной оси shape
       shape - основное тело
       width - ширина пластины
       angle - угол начала сегмента относительно оси Z в градусах
       pos - положение сегмента относительно центра масс shape
       height - толщина сегмента
    '''
    edge = findMainAxis(shape)
    axis = edge.Vertexes[1].Point - edge.Vertexes[0].Point

    if not radius:
        lmin, lmax = shapeSize(shape)
        radius = max(abs(lmin.z),abs(lmin.y), abs(lmax.z),abs(lmax.y)) + 1

    s = Part.makeBox(width, radius, height)
    s.Placement.Rotation = Rotation(Vector(1,0,0), axis)
    rotateL(s, (0,1,0), 180)
    rotateL(s, (1,0,0), angle)
    s.Placement.Base = edge.CenterOfMass + axis.normalize() * pos
    moveL(s, (0,0,1), -height/2)

    return s
#---------------------------------------------------------------
def createCutPie(shape, segment=45, angle=0, pos=0, height=2, axis=None, radius=None, placement=None):
    '''Создает сегмент который режет shape перпендикулярно оси shape
       shape - основное тело
       segment - угловой размер сегмента в градусах
       angle - угол начала сегмента относительно оси Z в градусах
       pos - положение сегмента относительно центра масс shape
       height - толщина сегмента
       axis - основная ось shape
       radius -
    '''
    if not placement:
        placement = shape.Placement

    if not axis:
        edge = findMainAxis(shape)
        axis = edge.Vertexes[1].Point - edge.Vertexes[0].Point

    if not radius:
        lmin, lmax = shapeSize(shape)
        radius = max(lmax.y-lmin.y, lmax.z-lmin.z)/2

    if height:
        s = Part.makeCylinder(radius, height, Vector(0,0,0), Vector(0,0,1), segment)
    else:
        s = Part.makeCircle(radius, Vector(0,0,0), Vector(0,0,1), 0, segment)
    s.Placement.Rotation = Rotation(Vector(0,0,1), axis)
    ax,ay,az = getAxes(placement)
    sax,say,saz = getAxes(s)
    rotateL(s, (0,0,1), ay.getAngle(sax)*360/(2*math.pi))
    rotateL(s, (1,0,0), 180)
    s.Placement.Base = edge.CenterOfMass
    moveL(s, (0,0,1), pos-height/2)

    return s
#---------------------------------------------------------------
def createAlonePlane(shape, angleZ = 0, size=100):
    ''' Создат плоскость проходящую через ось объекта
        под углом angleZ к оси Z
    '''
    # определение вектора основной оси объекта
    axis = findMainAxis(shape)
    dirX = (axis.Vertexes[1].Point - axis.Vertexes[0].Point).normalize()

    plane = Part.makePlane(size, size)
    plane.Placement.Rotation = Rotation(Vector(1,0,0), dirX)
    plane.Placement.Base = axis.CenterOfMass - dirX * size/2

    rotateL(plane, (1,0,0), angleZ)

    return plane
#---------------------------------------------------------------
def createCrossPlane(shape, pos=0, size=100):
    ''' createCrossPlane(shape, pos=0, size=100)
        Создает плоскость пересекающую объект перпендикулярно в центре

    '''
    axis = findMainAxis(shape)
    dir = (axis.Vertexes[1].Point - axis.Vertexes[0].Point).normalize()

    p = Part.makePlane(size, size)
    p.Placement.Rotation = Rotation(Vector(0,0,1), dir)
    p.Placement.Base = axis.CenterOfMass-p.CenterOfMass

    sax, say, saz = getAxes(shape)
    pax, pay, paz = getAxes(p)
    rotateL(p, (0,0,1), say.getAngle(pay)*360/(math.pi*2), p.CenterOfMass)
    moveL(p, (0,0,1), pos)

    return p
#===============================================================
# Операции в локальной системе координат
#===============================================================
def moveL(shape, dir, len):
    ''' moveL(shape, dir, len)
        Относительное перемещение в локальной системе координат
        shape - геометрический объект
        dir - направление перемещения в ЛСК
        len - длина перемещения
    '''
    if dir.__class__.__name__ in ('list', 'tuple'):
        dir = Vector(dir)

    if dir.__class__.__name__ == 'tuple':
        dir = Vector(dir)

    # Преобразование dir в ЛСК к ГСК
    v = (calcL2G(shape.Matrix, dir) - shape.Placement.Base).normalize()

    shape.translate(v * len)
#---------------------------------------------------------------
def rotateL(shape, axisL, angle, pointL=None):
    ''' rotateL(shape, axisL, angle, pointL=None)
        Относительный поворот вокруг локальной оси объекта
        shape - геометрический объект
        axisL - ось вокруг которой поворачиваем Vector или tuple
        angle - угол поворота в градусах
        pointL - точка поворота, по умолчанию начало ЛСК
    '''
    if not pointL:
        pointL = shape.Placement.Base

    if axisL.__class__.__name__ == 'tuple':
        axisL = Vector(axisL)

    axisG = shape.Matrix.multVec(axisL) - shape.Placement.Base
    shape.rotate(pointL, axisG, angle)
#---------------------------------------------------------------
def getL2Gmatrix(shape):
    '''Создание матрицы преобразования из локальных координат в глобальные
       shape - объект типа Part.Solid или Base.Placement
    '''
    if type(shape)==Part.Solid:
        return shape.Matrix

    if shape.__class__.__name__=="Placement":
        return shape.toMatrix()

    #(ax,ay,az) = getAxes(shape)
    #px = shape.Base.x
    #py = shape.Base.y
    #pz = shape.Base.z

    #m=FreeCAD.Matrix(
    #    ax.x, ax.y, ax.z, px,
    #    ay.x, ay.y, ay.z, py,
    #    az.x, az.y, az.z, pz,
    #    0,    0,    0,    1)

    #return m
#---------------------------------------------------------------
def calcL2G(matrix, vector):
    ''' Преобразование локальных координат к глобальным через матрицу L2G'''
    return matrix.multVec(vector)
#---------------------------------------------------------------
def calcG2L(matrix, vector):
    ''' Преобразование глобальных координат к локальным через матрицу L2G'''
    return matrix.inverse().multVec(vector)
#===============================================================
# Характеристики объектов
#===============================================================
def shapeSize(shape):
    xmin=None;xmax=None;ymin=None;ymax=None;zmin=None;zmax=None
    for v in shape.Vertexes:
        vl = calcG2L(shape.Matrix, v.Point)
        if xmax==None or vl.x > xmax: xmax = vl.x
        if xmin==None or vl.x < xmin: xmin = vl.x
        if ymax==None or vl.y > ymax: ymax = vl.y
        if ymin==None or vl.y < ymin: ymin = vl.y
        if zmax==None or vl.z > zmax: zmax = vl.z
        if zmin==None or vl.z < zmin: zmin = vl.z

    return Vector(xmin, ymin, zmin), Vector(xmax,ymax,zmax)
#---------------------------------------------------------------
def planeNormal(plane):
    return plane.PrincipalProperties['FirstAxisOfInertia']
#---------------------------------------------------------------
def distVertexToPlanes(vertex, planes):
    ''' distVertexToPlanes(vertex, planes[])
        Возвращает массив расстояний от vertex до planes
    '''
    dist = []

    for p in planes:
        norm = p.Surface.Axis
        point = p.CenterOfMass
        dist.append(round(vertex.Point.distanceToPlane(point, norm),2))

    return dist
#---------------------------------------------------------------
def checkObjBetweenPlanes(mainShape, shape, planes):
    ''' checkObjBetweenPlanes(obj, planes[])
        Проверяет находится ли obj между planes
    '''
    # Определяем плоскости которые не соосны объекту
    # - если пересечение главной оси объекта с плоскостью дает одну точку
    ma = findMainAxis(mainShape)
    crossPlanes = []
    for i in range(len(planes)):
        if len(planes[i].section(ma).Vertexes) == 1:
            crossPlanes.append(i)

    for v in shape.Vertexes:
        # Проверяем чтобы точка находилась между
        # - соосными плоскостями по счетчику ca
        # - секущими плоскостями по счетчику cc
        d = distVertexToPlanes(v, planes)
        ca = 1  # счетчик ориентации к соосным плоскостям
        cc = 1  # счетчик ориектации к не соосным плоскостям
        for i in range(len(d)):
            if i in crossPlanes:
                cc *= d[i]
            else:
                ca *= d[i]

        # Если знак положительный значит были два множителя одного знака
        if ca>0 or cc>0:
            return False

    return True
#---------------------------------------------------------------
def getSelectionPlacement(index=0):
    '''Возвращает Placement в глобальных координатах для выбранного объекта
       index - номер выбранного объекта в Gui.Selection.getSelection()[index]
    '''
    return Gui.Selection.getSelection()[index].getGlobalPlacement()
#---------------------------------------------------------------
def getAxes(shape):
    '''Возвращает три локальных оси объекта
    '''
    if type(shape) in (Part.Solid, Part.Face, Part.Shape):
        ax=shape.Matrix.multVec(Vector(1,0,0))-shape.Placement.Base
        ay=shape.Matrix.multVec(Vector(0,1,0))-shape.Placement.Base
        az=shape.Matrix.multVec(Vector(0,0,1))-shape.Placement.Base
    elif shape.__class__.__name__=="Placement":
        ax=shape.multVec(Vector(1,0,0))-shape.Base
        ay=shape.multVec(Vector(0,1,0))-shape.Base
        az=shape.multVec(Vector(0,0,1))-shape.Base
    else:
        print("Незвестный тип фигуры", type(shape), 'name=',shape.__class__.__name__)
        pass
        #ax=shape.PrincipalProperties['SecondAxisOfInertia']
        #ay=shape.PrincipalProperties['ThirdAxisOfInertia']
        #az=shape.PrincipalProperties['FirstAxisOfInertia']

    return (ax,ay,az)
#---------------------------------------------------------------
def findMainAxis(shape):
    ''' Определяет основную ось объекта
        возвращает объект типа TopoShape
    '''
    # Список окружностей образующих объект
    ca = []
    for e in shape.Edges:
        if e.Length > 0.1:
            c = e.Curve
            if c.__class__.__name__ == 'Circle':
                i = {'R':c.Radius, 'C':c.Center, 'A':c.Axis}
                ca.append(i)

    # Выбираем пару окружностей:
    #  - расстояние между нормалями < 0.1
    #  - расстояние между центрами > 0.1
    pairs = []
    for i in range(len(ca)):
        for j in range(i+1, len(ca)):
            if ca[i]['A'].distanceToPoint(ca[j]['A']) < 0.1 and ca[i]['C'].distanceToPoint(ca[j]['C']) > 0.1:
                pairs.append([ca[i]['R']+ca[j]['R'], i, j])

    if len(pairs) < 2:
        return Part.makeLine(shape.Placement.Base, shape.Placement.Base + shape.Placement.multVec(Vector(1,0,0)))

    # Выбираем пару окружностей по максимальному суммарному радиусу
    pairs.sort()
    # Строим линию между центрами окружностей - это ось объекта
    axis = Part.makeLine(ca[pairs[-1][1]]['C'], ca[pairs[-1][2]]['C'])
    return axis
#===============================================================
# Отладочные функции
#===============================================================
def showSection(shape, tools, name=None):
    if not name: name = 'Section'

    s = shape.section(tools)
    Part.show(s, name)
    o = App.ActiveDocument.Objects[-1]
    o.ViewObject.LineWidth = 3

    return s
#-----------------------------------------------------------
def showLBoundBox(shape):
    print('BoundBox')
    (lmin, lmax) = shapeSize(shape)

    gmin = calcL2G(shape.Matrix, lmin)
    gmax = calcL2G(shape.Matrix, lmax)

    xs = max(lmax.x-lmin.x,0.1)
    ys = max(lmax.y-lmin.y,0.1)
    zs = max(lmax.z-lmin.z,0.1)

    s = Part.makeBox(xs, ys, zs)
    s.transformShape(m.inverse())
    s.translate(gmin)

    Part.show(s)
    App.ActiveDocument.Objects[-1].ViewObject.Transparency = 80
    return s
#---------------------------------------------------------------
def showAxes(shape, size=10):
    (ax,ay,az) = getAxes(shape)

    showVector(shape.Placement.Base, ax*size, 'axisX', (1.0,0.0,0.0))
    showVector(shape.Placement.Base, ay*size, 'axisY', (0.0,1.0,0.0))
    showVector(shape.Placement.Base, az*size, 'axisZ', (0.0,0.0,1.0))
#---------------------------------------------------------------
def showVector(point, vector, name=None, color=None):
    ''' showVector(startPoint, Vector)
        показывает Vector из startPoint
    '''
    line = Part.makeLine(point, point+vector)
    if name:
        Part.show(line, name)
    else:
        Part.show(line)

    if color:
        App.ActiveDocument.Objects[-1].ViewObject.LineColor = color
#---------------------------------------------------------------
def show(obj, name=None, size=3):
    '''Показвает фигуру
       для Vertex показывает сферу
    '''
    c = obj.__class__.__name__

    pnt = None
    if c == 'Vertex':
        pnt = obj.Point
    elif c == 'Vector':
        pnt = obj
    elif c == 'Solid':
        pnt =  obj.Placement.Base

    if not pnt:
        print('Object class', c, 'not supported')
        return

    p = Part.makeSphere(size, pnt)

    if name:
        Part.show(p, name)
    else:
        Part.show(p)

    o = App.ActiveDocument.Objects[-1]
    o.ViewObject.PointSize = size
    o.ViewObject.LineWidth = size/2
#---------------------------------------------------------------
def showShapes(shapes):
    '''Показывает фигуры из массива
       для точек создает сферы с радиусом=3
    '''
    if type(shapes) not in (list, tuple): shapes = [shapes]

    for s in shapes:
        if s.__class__.__name__=='Vector':
            Part.show(Part.makeSphere(3, s), 'Point')
        else:
            Part.show(s, s.ShapeType)
#---------------------------------------------------------------
def showEdge(e, Name):
    ''' Создает линию соответствующую Edge '''

    v = e.discretize(Deflection=0.1, First=e.FirstParameter, Last=e.LastParameter)

    try:
        p = Part.makePolygon(v)
    except Exception as e:
        print('Error v=',v)
        return

    if Name:
        Part.show(p, Name)
    else:
        Part.show(p)

    obj = App.ActiveDocument.Objects[-1]
    obj.ViewObject.PointSize = 10
    return
#---------------------------------------------------------------
def showEdges(es):
    ''' Создает линии соответствующие Edges '''

    c = 0;
    for e in es:
        showEdge(e, 'Edge' + str(c))
        c+=1
#---------------------------------------------------------------
def _str(obj, short=False, origin=None):
    t = obj.__class__.__name__
    r = ''

    if t in ('str'):
        return '"' + obj + '"'

    if t == 'int':
        return str(obj)

    if t == 'float':
        return '{:.1f}'.format(obj)

    if t == 'tuple':
        for i in range(len(obj)):
            r += _str(obj[i], origin=origin)+', '
        if len(r)>2: r = r[:-3]
        return '(' + r + ')'

    if t == 'list':
        for i in range(len(obj)):
            r += '{:2d}'.format(i) + ' : ' + _str(obj[i],origin=origin)+',\n'
        if len(r)>2: r = r[:-3]
        return '[' + r + ']\n'

    if t == 'dict':
        for k,v in obj: r += '"'+k+'":' + _str(v, origin=origin)+', '
        if len(r)>2: r = r[:-3]
        return '{' + r + '}\n'

    if t == 'Vector':
        x = obj.x - (origin.x if origin else 0)
        y = obj.y - (origin.y if origin else 0)
        z = obj.z - (origin.z if origin else 0)
        r = '(' + _str(x) + ', ' + _str(y) + ', ' + _str(z) + ')'
        if not short: r = 'Vector' + r
        return r

    if t == 'Vertex':
        r += 'Vertex'
        r += _str(obj.Point, True, origin)
        return r

    if t == 'Edge':
        r += 'Edge(Length:' + _str(obj.Length) + ' '
        r += '(' + _str(obj.valueAt(obj.FirstParameter), True, origin) + ')'
        r += '-'
        r += '(' + _str(obj.valueAt(obj.LastParameter), True, origin) + ')'
        return r

    if t == 'Face':
        r = 'Face(Area:' + _str(obj.Area)
        r += '  Orient.:'+ obj.Orientation
        r += '  Surf.axis:' + _str(obj.Surface.Axis, True)
        r += ')'
        return r

    if t == 'Wire':
        r = 'Wire(' + \
            ' Length:' + _str(obj.Length) + \
            ' Edges:' + len(obj.Edges) + ')'
        return r

    return 'class ' + t
#---------------------------------------------------------------
def list(obj, origin=None):
    if origin: print('origin:', _str(origin, True))
    print(_str(obj, origin=origin))
#---------------------------------------------------------------
def listNormales(obj):
    t = obj.__class__.__name__

    if t == 'Face':
        umin,umax,vmin,vmax = obj.ParameterRange
        list((umin, vmin, obj.normalAt(umin, vmin)))
        list((umin, vmax, obj.normalAt(umin, vmax)))
        list(((umin+umax)/2, (vmin+vmax)/2, obj.normalAt((umin+umax)/2, (vmin+vmax)/2)))
        list((umax, vmin, obj.normalAt(umax, vmin)))
        list((umax, vmax, obj.normalAt(umax, vmax)))

    else:
        print('Unknown type: ', t)
#---------------------------------------------------------------
def listAxes(shape):
    (ax, ay, az) = getAxes(shape)
    print('axisX', _str(ax, True))
    print('axisY', _str(ay, True))
    print('axisZ', _str(az, True))
#---------------------------------------------------------------
def listVertexes(shape):
    for v in shape.Vertexes:
        print (v.Point)
#---------------------------------------------------------------
def info(a, all=False, Type='', Name=''):
  print('==============================')
  print('class:', a.__class__.__name__)
  print('type:', type(a))
  print(a.__doc__)
  print('------------------------------')
  lst = dir(a)
  for i in lst:
    if i in ('__doc__','__dir__','__class__','__sizeof__'):
      continue
    try:
        v = getattr(a,i)
        t = v.__class__.__name__
    except Exception as e:
        print(i,'\terror')
        continue
    if 'method-wrapper' in t:
      continue
    if Type and not(Type.lower() in t.lower()):
      continue
    if Name and not(Name.lower() in i.lower()):
      continue
    tabs = '\t'
    if len(i) < 9:
      tabs += '\t'
    if t in ('float', 'bool', 'int'):
      print(i, tabs, v)
    elif i in ('Content','Annotation', 'IV'):
        print(i)
    elif t in ('str'):
      print(i, tabs+'"' + v + '"')
    elif t in ('list', 'tuple'):
        if len(v)==0 or v[0].__class__.__name__ in ('float','int','str', 'Vector'):
          print(i, tabs, v)
        else:
          print(i, tabs, len(v), ' of ', type(v[0]))
    elif t in ('dict'):
      print(i, tabs, v)
    elif t == 'Vector':
        print(i, tabs ,'x={:.2f} y={:.2f} z={:.2f}'.format(v.x, v.y, v.z))
    elif t in ('Rotation', 'Placement', 'BoundBox','Quantity'):
        print(i, tabs ,v)
    else:
      if all or Type or Name or (not('built' in t) and not('wrapper' in t) and not('__' in i)):
        print(i, tabs, type(v))
  return a
#---------------------------------------------------------------
