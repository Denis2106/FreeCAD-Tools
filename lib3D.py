'''
Библиотека для работы с трехмерными объектами

Классы
------
Vector3D        - вектор в трехмерном пространстве
Quaternion      - кватернион
Matrix3D        - матрица для преобразований в трехмерном пространстве
'''
import math
import numpy as np

#======================================================================
# Функции работы с векторами
#======================================================================
# Класс для работы с векторами в 3-х мерном пространстве
class Vector3D:
    '''
    Вектор в 3-х мерном пространстве
    '''

    def __init__(self, xyz=None, y=None, z=None):
        if xyz == None:
            self.x = self.y = self.z = 0

        elif isinstance(xyz, Vector3D):
            self.x, self.y, self.z = xyz.pos()

        elif y!=None and z!=None:
            self.x, self.y, self.z = xyz, y, z

        elif type(xyz) in (tuple, list):
            self.x, self.y, self.z = xyz[0], xyz[1], xyz[2]

        elif getattr(xyz,'x', None)!=None and getattr(xyz,'y',None)!=None and getattr(xyz,'z')!=None:
            self.x, self.y, self.z = xyz.x, xyz.y, xyz.z

        else:
            raise Exception('Vector3D: неверные параметры (%s, %s, %s)' % \
                            (str(xyz), str(y), str(z)) )

    # Возвращает три координаты в виде кортежа
    def pos(self):
        return (self.x, self.y, self.z)

    # Текстовое представление объекта
    def __repr__(self):
        return 'Vector3D(%.2f, %.2f, %.2f)' % self.pos()

    # длина вектора
    def len(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    # Нармализованный вектор
    def norm(self):
        res = Vector3D()
        l = self.len()
        if l > 0:
            res.x = self.x / l
            res.y = self.y / l
            res.z = self.z / l
        return res

    def __neg__(self):
        return Vector3D(-self.x, -self.y, -self.z)

    # Сложение векторов
    def __add__(self, operand):
        res = Vector3D(self)
        operand = Vector3D(operand)
        res.x += operand.x
        res.y += operand.y
        res.z += operand.z
        return res

    # Вычитание векторов
    def __sub__(self, operand):
        res = Vector3D(self)
        operand = Vector3D(operand)
        res.x -= operand.x
        res.y -= operand.y
        res.z -= operand.z
        return res

    # Векторное умножение на вектор или скаляр
    def __mul__(self, operand):
        res = Vector3D()
        if type(operand) in (int, float):
            # Произведеление вектора на скаляр
            res.x = self.x * operand
            res.y = self.y * operand
            res.z = self.z * operand
        elif isinstance(operand, Vector3D):
            # Произведение векторов
            a = self
            b = Vector3D(operand)
            res.x = a.y*b.z - a.z*b.y
            res.y = a.z*b.x - a.x*b.z
            res.z = a.x*b.y - a.y*b.x
        else:
            res = operand.__rmul__(self)

        return res

    # Скалярное умножение
    def dot(self, vector):
        if isinstance(vector, Vector3D):
            vector = Vector3D(vector)

        return self.x*vector.x + self.y*vector.y + self.z*vector.z

    # Угол между векторами
    def angle(self, operand):
        # все координаты в глобальной системе
        # b - данный вектор
        # c - второй вектор
        # a - вектор противолежащий углу равный раззностному вектору
        # угол определяется по теореме косинусов
        operand = Vector3D(operand)
        b = self.len()
        c = operand.len()
        a = (operand - self).len()

        if not c:
            raise Exception('Vector3D длина вектора %s равна 0' % str(operand))

        angle = math.acos((b**2 + c**2 - a**2)/(2*b*c)) * 180 / math.pi
        return angle

    # Проекция на вектор
    def projection(self, *axis):
        # проекция = длина данного * cos угла * нормированный вектор оси
        axis = Vector3D(*axis).norm()
        angle = self.angle(axis)
        l = self.len()
        return axis * (l * math.cos(angle/180*math.pi))

    # Вектор > вектора-аргумента
    def __gt__(self, vector):
        return self.len() > vector.len()

#======================================================================
# Функции работы с кватернионами
#======================================================================
# Конвертация кватерниона в ось и угол
def Q2R(x, y, z, w):
    scale = math.sqrt(x*x + y*y + z*z)

    if scale==0: return Vector3D(), 0

    ax = x/scale
    ay = y/scale
    az = z/scale
    angle = 2 * math.atan2(scale, w)
    if w < 0: angle = -angle

    return Vector3D(ax,ay,az), angle*180/math.pi

# Конвертация матрицы в кватернион
def M2Q(matrix):
    m = Matrix3D(matrix).matrix

    tr = m[0,0] + m[1,1] + m[2,2]

    if tr > 0.0001:
        s = math.sqrt(tr + 1)
        w = s / 2
        s = 0.5 / s
        x = (m[1,2] - m[2,1]) * s
        y = (m[2,0] - m[0,2]) * s
        z = (m[0,1] - m[1,0]) * s
    else:
        if m[0,0] > max(m[1,1], m[2,2]):
            # Наибольший элемент следа [0,0]
            i = 0; j=1; k=2
        elif m[1,1] > max(m[0,0], m[2,2]):
            # Наибольший элемент следа [1,1]
            i = 1; j=2; k=0
        else:
            # Наибольший элемент следа [2,2]
            i = 2; j=0; k=1

        q = [0,0,0,0]
        s= math.sqrt(m[i,i] - m[j,j] - m[k,k] + 1)
        q[i] = s * 0.5
        if s != 0: s = 0.5 / s
        q[3] = (m[j,k] - m[k,j]) * s
        q[j] = (m[i,j] + m[j,i]) * s
        q[k] = (m[i,k] + m[k,i]) * s

        x, y, z, w = q

    return x, y, z, w

# Преобразование кватерниона в матрицу
def Q2M(*quat):
    quat = Quaternion(*quat)

    x2 = quat.x + quat.x
    y2 = quat.y + quat.y
    z2 = quat.z + quat.z

    xx = quat.x*x2; xy = quat.x*y2; xz = quat.x*z2
    yy = quat.y*y2; yz = quat.y*z2; zz = quat.z*z2
    wx = quat.w*x2; wy = quat.w*y2; wz = quat.w*z2

    m = Matrix3D()
    m.setRow(0, ( 1-(yy+zz),     xy-wz,     xz+wy ))
    m.setRow(1, (     xy+wz, 1-(xx+zz),     yz-wx ))
    m.setRow(2, (     xz-wy,     yz+wx, 1-(xx+yy) ))

    return m

# Векторное произведение кватернионов
def QQ(q1, q2):
    A = (q1.w + q1.x) * (q2.w + q2.x)
    B = (q1.z - q1.y) * (q2.y - q2.z)
    C = (q1.x - q1.w) * (q2.y + q2.z)
    D = (q1.y + q1.z) * (q2.x - q2.w)
    E = (q1.x + q1.z) * (q2.x + q2.y)
    F = (q1.x - q1.z) * (q2.x - q2.y)
    G = (q1.w + q1.y) * (q2.w - q2.z)
    H = (q1.w - q1.y) * (q2.w + q2.z)

    q = Quaternion()
    q.w =  B + (-E -F +G +H) * 0.5
    q.x =  A - ( E +F +G +H) * 0.5
    q.y = -C + ( E -F +G -H) * 0.5
    q.z = -D + ( E -F -G +H) * 0.5

    return q

#======================================================================
# Класс для работы с кватернионами
#======================================================================
# Класс для работы с кватернионами
class Quaternion:
    # Варианты вызова конструктора
    # Quaternion()
    # Quaternion( quaternion )
    # Quaternion( matrix )
    # Quaternion( (x,y,z,w) )
    # Quaternion( x,y,z,w )
    def __init__(self, arg=None, y=None, z=None, w=None):
        if arg == None:
            self.x = self.y = self.z = 0
            self.w = 1

        elif isinstance(arg, Quaternion):
            self.copy(arg)

        elif isinstance(arg, Matrix3D):
            r = M2Q(arg)
            self.x = -r[0]
            self.y = -r[1]
            self.z = -r[2]
            self.w = r[3]

        elif type(arg) in (tuple, list):
            self.x = arg[0]
            self.y = arg[1]
            self.z = arg[2]
            self.w = arg[3]

        elif type(arg) in (float, int):
            self.x = arg
            self.y = y
            self.z = z
            self.w = w

        else:
            raise Exception('Quaternion: неверные параметры (%s, %s, %s, %s)' % \
                            (str(arg), str(y), str(z), str(w)) )

    # Текстовое представление кватерниона
    def __repr__(self):
        return 'Quaternion(%.2f, %.2f, %.2f, w=%.2f)' % \
                (self.x, self.y, self.z, self.w)

    # Создает копию себя, если указан аргумент, то копирует себя из аргумента
    def copy(self, quat=None):
        if quat == None:
            q = Quaternion(self.x, self.y, self.z, self.w)
            return q
        else:
            self.x = quat.x
            self.y = quat.y
            self.z = quat.z
            self.w = quat.w
            return self

    # устанавливает атрибуты x,y,z из вектора
    def setVector(self, vector):
        if not isinstance(vector, Vector3D):
            vector = Vector3D(vector)
        self.x = vector.x
        self.y = vector.y
        self.z = vector.z

    # возвращает кортеж (x, y, z, w)
    def data(self):
        return (self.x, self.y, self.z, self.w)

    # Длина кватерниона
    def length(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2 + self.w**2)

    # Возвращает вектор и угол поворота
    def rotation(self):
        return Q2R(self.x, self.y, self.z, self.w)

    # Возвращает ось поворота
    def rotationAxis(self):
        v,a = self.rotation()
        return v

    # Вощвращает угол поворота
    def rotationAngle(self):
        v,a = self.rotation()
        return a

    # Возвращает Matrix3D созданный из кватерниона
    def toMatrix(self):
        return Q2M(self)

    # Сложение кватернионов
    def __add__(self, operand):
        if isinstance(operand, Quaternion):
            res = Quaternion()
            res.x = self.x + operand.x
            res.y = self.y + operand.y
            res.z = self.z + operand.z
            res.w = self.w + operand.w
            return res
        else:
            raise Exception('Сложение кватерниона с операндом типа %s не определено' % str(type(operand)))

    # Векторное умножение кватернионов
    def __mul__(self, operand):
        if isinstance(operand, Quaternion):
            # Перемножение кватернионов
            return QQ(self, operand)

        elif type(operand) in (float, int):
            # Умножение кватерниона на скаляр
            r = self.copy()
            r.x *= operand
            r.y *= operand
            r.z *= operand
            r.w *= operand
            return r

        else:
            raise Exception('Умножение кватерниона на операнд типа %s не определено' % str(type(operand)))

    # Скалярное произведение кватернионов
    def dot(self, operand):
        return self.x*operand.x + self.y*operand.y + self.z*operand.z + self.w*operand.w

    def __sub__(self, operand):
        return (~operand) * self

    # Нормализация кватерниона, меняет себя
    def normal(self, selfModify=False):
        l = self.length()
        r = self.copy()
        if l > 0: r = r * (1/r.length())
        if selfModify: self.copy( r )
        return r

    # Вычисление сопряженного вектора
    def conjugate(self, selfModify=False):
        res = Quaternion()
        res.x = - self.x
        res.y = - self.y
        res.z = - self.z
        res.w = self.w
        if selfModify: self.copy(res)
        return res

    def __invert__(self):
        mq = self.conjugate()
        return mq * (1/(mq.x*mq.x + mq.y*mq.y + mq.z*mq.z + mq.w*mq.w))

    def angle(self, operand):
        cos_angle = self.dot(operand) * (1/ (self.length()*operand.length()))
        return math.acos(cos_angle) * 180 / math.pi

    # Устанавливает значение кватерниона = поворот на angle вокруг vector
    def rotateTo(self, vector, angle):
        vector = Vector3D(vector).norm()
        self.w = math.cos( angle/2 * math.pi/180)
        vector = vector * math.sin( angle/2 * math.pi/180)
        self.setVector(vector)
        return self

    # Дополнительный поворот кватерниона на angle вокруг vector
    def rotateInc(self, vector, angle):
        q = Quaternion()
        q.rotateTo(vector, angle)
        self.copy( self * q )
        return self

    # Применемение кватерниона к вектору
    def useFor(self, vector):
        qv = Quaternion()
        qv.setVector(vector)
        rqv = self * qv * (~self)
        return Vector3D(rqv.x, rqv.y, rqv.z)

    # Установка из углов Эйлера
    def setRPY(self, xa, ya, za):
        self.copy(Quaternion())
        self.rotateTo((1,0,0), xa).rotateInc((0,1,0),ya).rotateInc((0,0,1),za)
        return self

    # Параметризация перехода от self к q2 для точки d = 0..1
    def SLERP(self, q2, d):
        if d > 0.001:
            if angle > 90: angle = 180 - angle
            angle = self.angle(q2) * math.pi / 180
            res = self * math.sin((1-d)*angle) + q2 * math.sin(d*angle)
            res = res * (1/math.sin(angle))
        else:
            res = self.copy()

        return res


#======================================================================
# Класс для работы с матрицами
#======================================================================
class Matrix3D():
    # Конструктор
    def __init__(self, matrix=None):
        if not matrix:
            self.matrix = np.eye(4)

        elif isinstance(matrix, Matrix3D):
            self.matrix = matrix.matrix.copy()

        elif matrix.__class__.__name__ == 'Matrix':
            # Аргумент - матрица FreeCAD
            self.matrix = np.eye(4)
            for r in range(4):
                for c in range(4):
                    # Загрузка с транспонированием
                    self.matrix[r,c] = matrix.A[r+c*4]

    def setRow(self, row, arg):
        # Заполняет заданную строку матрицы
        if isinstance(arg, Vector3D): arg = (arg.x, arg.y, arg.z)
        for i in range(len(arg)):
            self.matrix[row, i] = arg[i]

    def FreeCADmatrix(self):
        matrix = self.matrix.transpose()
        res = []
        for r in range(4):
            for c in range(4):
                res.append(matrix[r,c])
        return res

    def transpose(self):
        res = Matrix3D()
        res.matrix = self.matrix.transpose()
        return res

    # Текстовое представление объекта
    def __repr__(self):
        return 'Matrix3D(%s)' % str(self.matrix.tolist())

    # Операция умножения, объект слева
    def __mul__(self, operand):
        res = Matrix3D()
        res.matrix = np.dot(self.matrix, operand.matrix)
        return res

    # Операция умножения, объект справа
    def __rmul__(self, vector):
        if not isinstance(vector, Vector3D):
            vector = Vector3D(vector)
        res =  np.dot([vector.x, vector.y, vector.z, 1], self.matrix)
        return Vector3D(res[0], res[1], res[2])

    # Операция инверсии "~"
    def __invert__(self):
        res = Matrix3D()
        res.matrix = np.linalg.inv(self.matrix)
        return res

    # Матрица перемещения на абсолютное значение
    def moveTo(self, *vector):
        self.matrix = np.eye(4)
        return self.moveInc(*vector)

    # Добавление перемещения к существующей матрице
    def moveInc(self, *vector):
        if not isinstance(vector, Vector3D):
            vector = Vector3D(*vector)
        m = np.eye(4)
        m[3,0], m[3,1], m[3,2] = vector.pos()
        self.matrix = np.dot(self.matrix, m)
        return self

    # Создание матрицы поворота
    def rotateTo(self, axis, angle):
        self.matrix = np.eye(4)
        return self.rotateInc(axis, angle)

    # Добавление поворота к существующей матрице
    def rotateInc(self, axis, angle):
        axis = Vector3D(axis)
        x, y, z = axis.pos()

        a = angle / 180 * math.pi
        cos = math.cos(a)
        sin = math.sin(a)

        m = np.eye(4)
        m[0,0] = cos + (1-cos)*x*x
        m[0,1] = (1-cos)*x*y - sin*z
        m[0,2] = (1-cos)*x*z + sin*y
        m[1,0] = (1-cos)*y*x + sin*z
        m[1,1] = cos + (1-cos)*y*y
        m[1,2] = (1-cos)*y*z - sin*x
        m[2,0] = (1-cos)*z*x - sin*y
        m[2,1] = (1-cos)*z*y + sin*x
        m[2,2] = cos + (1-cos)*z*z

        self.matrix = np.dot(self.matrix, m)
        return self

# TODO Точка начала координат
class Origin3D():

    def __init__(self, base=None, axisX=None, axisZ=None):
        if base:
            self.base = Vector3D(base)
        else:
            self.base = Vector3D(0,0,0)

        if axisX:
            self.axisX = Vector3D(axisX)
        else:
            self.axisX = Vector3D(1,0,0)

        if axisZ:
            self.axisZ = Vector3D(axisZ)
        else:
            self.axisZ = Vector3D(0,0,1)

        self.origin = None

    def __repr__(self):
        return 'Origin3D(base=(%.0f, %.0f, %.0f), ' \
                        'axisX=(%.0f, %.0f, %.0f), ' \
                        'axisZ=(%.0f, %.0f, %.0f))' % \
                        (self.base.x, self.base.y, self.base.z,
                        self.axisX.x, self.axisX.y, self.axisX.z,
                        self.axisZ.x, self.axisZ.y, self.axisZ.z)

    # Установить систему базовую систему отсчета
    def setOrigin(self, origin):
        self.origin = origin

    # Преобразование локальных координат в глобальные
    def L2G(self, *point):
        res = Vector3D(self.base)
        if not isinstance(point, Point3D):
            point = Point3D(*point)

        vx = self.axisX * point.x
        vy = (self.axisZ * self.axisX) * point.y
        vz = self.axisZ * point.z

        res += vx + vy + vz

        if self.origin:
            res = self.origin.L2G(res)

        return res

# Класс Shape - прообраз всех трехмерных объекттов
# наследники: Group, Point, Origin, Plane,
# Атрибуты:
# - group - группа к которой относится объект
# Методы:
# - работа с origin:  setOrigin, L2G
# - перемещение: moveTo, rotateTo
# - описание для отображения во FreeCAD
class Shape:
    def __init__(self):
        self.children = []       # дочерние объекты
        self.parent = None       # родительский объект
        self.origin = None       # база локальной системы координат (ЛСК)
        self.pos = Vector3D()    # позиция обхекта в ЛСК
        self.matrix = Matrix3D() # матрица преобразования объекта в ЛСК
        pass

    def addChild(self, child):
        self.children.appedn(child)
        child.parent = self
        child.origin = self.origin

    def setOrigin(self, origin):
        self.origin = origin
        for obj in self.children:
            obj.origin =origin

    def L2G(self, point):
        pass

    def moveInc(self, vector):
        pass

    def moveTo(self, point):
        pass

    def rotateInc(self, axis, angle):
        pass

    def rotateTo(self, axis, angle):
        pass

    # Описывет объект для отображения во FreeCAD d глобальной системе координат
    def self_content(self):
        return None

    def content(self):
        pass

# Точка
class Point3D():

    def __init__(self, *vector):
        if vector == None:
            self.x = 0
            self.y = 0
            self.z = 0
        elif isinstance(vector, Point3D):
            self.vector = Vector3D(vector.vector)
            self.x = vector.x
            self.y = vector.y
            self.z = vector.z
        else:
            self.vector = Vector3D(*vector)
            self.x = self.vector.x
            self.y = self.vector.y
            self.z = self.vector.z

    def __repr__(self):
        return 'Point3D(%.1f, %.1f, %.1f)' % (self.x, self.y, self.z)

# Отрезок
class Segment3D():
    # Segment3D(segment)
    # Segment3D(point or tuple, point or tuple)
    # Segment3D(point, vector)
    def __init__(self, arg1=None, arg2=None):
        if not arg1:
            self.p1 = Point3D()
            self.p2 = Point3D()

        elif isinstance(arg1, Segment3D):
            self.p1 = Point3D(arg1.p1)
            self.p2 = Point3D(arg1.p2)

        elif (isinstance(arg1, Point3D) or type(arg1) in (tuple,list)) and \
             (isinstance(arg2, Point3D) or type(arg2) in (tuple,list)):
            self.p1 = Point3D(arg1)
            self.p2 = Point3D(arg2)

        elif (isinstance(arg1, Point3D) or type(arg1) in (tuple,list)) and \
             isinstance(arg2, Vector3D):
            self.p1 = Point3D(arg1)
            self.p2 = Point3D(self.p1.vector + arg2)

        else:
            print('Неправильное сочетание аргументов')
            raise

    def __repr__(self):
        return 'Segment3D((%.f, %.0f, %.0f) - (%.f, %.0f, %.0f))' % \
                (self.p1.x, self.p1.y, self.p1.z,
                self.p2.x, self.p2.y, self.p2.z)

    def direction(self):
        return (Vector3D(self.p2) - Vector3D(self.p1)).norm()

    def len(self):
        return (Vector3D(self.p2) - Vector3D(self.p1)).len()

# Плоскость
class Plane():
    # Plane(plane)
    # Plane(point, normal)
    # Plane(point, point, point)
    def __init__(self, arg1=None, arg2=None, arg3=None):
        if not arg1:
            # По-умолчанию создается плоскость XY в точке (0,0,0)
            self.base = Point3D()
            self.normal = Vector3D(0,0,1)

        elif isinstance(arg1, Point3D) and isinstance(arg2, Vector3D):
            # Построение плоскости по нормали и точке на плоскости
            self.base = Point3D(arg1)
            self.normal = Vector3D(arg2)

        elif isinstance(arg1, Point3D) and isinstance(arg2, Point3D) and isinstance(arg3, Point3D):
            # Построение плоскости по трем точкам
            # расчитываются два вектора (с-a) и (c-b)
            # векторное проеизведение этих векторов дает перпендикуляр к плоскости
            v1 = arg3 - arg1
            v2 = arg3 - arg2
            self.nornal = (v1 * v2).norm()
            self.base = Point3D(arg1)

        else:
            raise Exception('Plane: неверные параметры (%s, %s, %s)' % \
                            (str(arg1), str(arg2), str(arg3)) )

    # Текстовое представление
    def __repr__(self):
        return 'Plane(base=(%.0f, %.0f, %.0f), normal=(%.0f, %.0f, %.0f))' % \
                (self.base.x, self.base.y, self.base.z,
                self.normal.x, self.normal.y, self.normal.z)

    # Проекция вектора на плоскость
    def projection_vector(self, vector):
        v = self.normal * vector
        if v.len()==0 and vector.len()!=0:
            # Вектор паралелен нормали плоскости, проекция равна нулю
            return Vector3D()
        else:
            v = v * self.normal
            return v
        # проекция = вектор - проекция ветора на перпендикуляр к плоскости



    # Дистанция от точки до плоскости
    def distance(self, point):
        v = Vector3D(point) - Vector3D(self.base)
        dist_vector = v - self.projection_point(point)
        sign = 1 if dist_vector + self.normal > self.normal else -1
        return dist_vector.len() * sign
