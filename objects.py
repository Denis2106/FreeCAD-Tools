'''
Объекты для отображения во FreeCAD

sys.path.append("c:\\Users\\dk274\\OneDrive\\dev\\Python\\FreeCAD")
'''

import FreeCAD as App
import Part
from FreeCAD import Base
from pivy import coin

RED =   (1.0, 0.0, 0.0)
GREEN = (0.0, 1.0, 0.0)
BLUE =  (0.0, 0.0, 1.0)

def objPaint(obj=None, color=RED):
    if obj == None:
        obj = App.ActiveDocument.Objects[-1]
    obj.ViewObject.ShapeColor = color

def makeArrow(base=(0,0,0), dir=(1,0,0), length=10, size=2, isShow=True, color=RED):
    if type(base) == tuple: base = App.Vector(base)
    if type(dir) == tuple: dir = App.Vector(dir)

    cone = Part.makeCone(0, size/3, size)
    cone.rotate((0,0,0), (1,0,0), 180)
    cone.Placement.Base.z = length

    axis = Part.makeCylinder(0.1, length-size)

    arrow = axis.fuse(cone)
    arrow.Placement.Rotation = App.Rotation(App.Vector(0,0,1), dir)
    arrow.Placement.Base = base

    if isShow:
        Part.show(arrow)
        objPaint(color=color)

    return arrow

def makeOrigin(shape, length=10, isShow=True):
    if type(shape) == str:
        shape = App.ActiveDocument.getObjectsByLabel(shape)[0]

    dirX = shape.Placement.Rotation.multVec(App.Vector(1,0,0))
    dirY = shape.Placement.Rotation.multVec(App.Vector(0,1,0))
    dirZ = shape.Placement.Rotation.multVec(App.Vector(0,0,1))

    base = shape.Placement.Base

    aX = makeArrow(base, dir=dirX, length=length, color=RED)
    aY = makeArrow(base, dir=dirY, length=length, color=GREEN)
    aZ = makeArrow(base, dir=dirZ, length=length, color=BLUE)

    origin = aZ.fuse([aX,aY])

    origin.Placement = shape.Placement

    if isShow: Part.show(origin)

    return origin


class Molecule:
	def __init__(self, obj):
		''' Add two point properties '''
		obj.addProperty("App::PropertyVector","p1","Line","Start point")
		obj.addProperty("App::PropertyVector","p2","Line","End point").p2=FreeCAD.Vector(5,0,0)

		obj.Proxy = self

	def execute(self, fp):
		''' Print a short message when doing a recomputation, this method is mandatory '''
		fp.Shape = Part.makeLine(fp.p1,fp.p2)

class ViewProviderMolecule:
	def __init__(self, obj):
		''' Set this object to the proxy object of the actual view provider '''
		sep1=coin.SoSeparator()
		self.trl1=coin.SoTranslation()
		sep1.addChild(self.trl1)
		sep1.addChild(coin.SoSphere())
		sep2=coin.SoSeparator()
		self.trl2=coin.SoTranslation()
		sep2.addChild(self.trl2)
		sep2.addChild(coin.SoSphere())
		obj.RootNode.addChild(sep1)
		obj.RootNode.addChild(sep2)
		# triggers an updateData call so the the assignment at the end
		obj.Proxy = self

	def updateData(self, fp, prop):
		"If a property of the handled feature has changed we have the chance to handle this here"
		# fp is the handled feature, prop is the name of the property that has changed
		if prop == "p1":
			p = fp.getPropertyByName("p1")
			self.trl1.translation=(p.x,p.y,p.z)
		elif prop == "p2":
			p = fp.getPropertyByName("p2")
			self.trl2.translation=(p.x,p.y,p.z)

	def __getstate__(self):
		return None

	def __setstate__(self,state):
		return None

def makeMolecule():
	doc=FreeCAD.newDocument()
	a=FreeCAD.ActiveDocument.addObject("Part::FeaturePython","Molecule")
	Molecule(a)
	ViewProviderMolecule(a.ViewObject)
	doc.recompute()
