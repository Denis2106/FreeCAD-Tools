import time
import FreeCAD as App
import FreeCADGui as Gui

def log(txt):
    App.Console.PrintLog(txt + '\n')


def disconnectAll():
    lst = list(App.Observers.values())
    for o in lst:
        o.disconnect()
        print('%s disconnected' % o.id)


class Observer():
    def __init__(self, typeId, ownerClass, isDebug=True):
        self.typeId = typeId
        if type(ownerClass) == str:
            self.owner = ownerClass
        else:
            self.owner = ownerClass.__class__.__name__

        self.id = self.typeId + '.' + self.owner
        self.callback = None
        self.history = []
        self.historyLen = 100
        self.isDebug = isDebug
        self.isPause = False


    def connect(self):
        if not getattr(App, 'Observers', None):
            App.Observers = {}

        if App.Observers.get(self.id, None):
            self.disconnect(App.Observers[self.id])

        App.Observers[self.id] = self

        if self.connectFunc:
            self.connectFunc(self)


    def disconnect(self, id=None):
        if not getattr(App, 'Observers', None): return

        if not id: id = self

        res = App.Observers.get(self.id, None)
        if res:
            del App.Observers[self.id]

        if self.disconnectFunc:
            self.disconnectFunc(res)


    def setCallback(self, callback):
        self.callback = callback

    def callCallback(self, *args):
        if self.callback:
            self.callback(*args)

    def signal(self, *args):
        if len(self.history) > self.historyLen:
            self.history = self.history[-self.historyLen:]
        self.history.append([time.time(), args])

        if self.isPause: return

        if self.isDebug:
            log('%s: %s( %s )\n' % (self.typeId, args[0], str(args[1:])))

        self.callCallback(*args)


class SelectionObserver(Observer):
    def __init__(self, ownerClass, isDebug=False):
        self.connectFunc = Gui.Selection.addObserver
        self.disconnectFunc = Gui.Selection.removeObserver
        super().__init__('SelectionObserver', ownerClass, isDebug)

    def setPreselection(self, doc, obj, sub):
        self.signal('setPreselection', doc, obj, sub)

    def addSelection(self, doc, obj,sub, pnt):
        self.signal('addSelection', obj, sub, pnt)

    def removeSelection(self, doc, obj, sub):
        self.signal('removeSelection', doc, obj, sub)

    def setSelection(self, doc):
        self.signal('setSelection', doc)

    def clearSelection(self, doc):
        self.signal('clearSelection', doc)


class AppObserver(Observer):
    # Актуализация github.com FreeCAD/src/App/DocumentObserverPython.h
    def __init__(self, ownerClass, isDebug=False):
        self.connectFunc = App.addDocumentObserver
        self.disconnectFunc = App.removeDocumentObserver
        super().__init__('AppObserver', ownerClass, isDebug)


    def slotCreatedObject(self, obj):
      self.signal('ObjCreated', obj)

    def slotDeletedObject(self, obj):
      self.signal('ObjDeleted', obj)

    def slotBeforeChangeObject(self, obj, prop):
      self.signal('ObjBeforeChange', obj, prop)

    def slotChangedObject(self, obj, prop):
      self.signal('ObjChanged', obj, prop)

    def slotCreatedDocument(self, doc):
      self.signal('DocCreated', doc)

    def slotDeletedDocument(self, doc):
      self.signal('DocDeleted', doc)

    def slotRelabelDocument(self, doc):
      self.signal('DocRelabled', doc)

    def slotActivateDocument(self, doc):
      self.signal('DocActivated', doc)

    def slotBeforeChangeDocument(self, doc, prop):
      self.signal('DocBeforeChange', doc, prop)

    def slotChangedDocuunment(self, doc, prop):
      self.signal('DocChanged', doc, prop)

    def slotUndoDocument(self, doc):
      self.signal('DocUndo', doc)

    def slotRedoDocument(self, doc):
      self.signal('DocRedo', doc)

    def slotRecomputedObject(self, obj):
      self.signal('ObjRecomputed', obj)

    def slotBeforeRecomputeDocument(self, doc):
        self.signal('DocBeforeChange', doc)

    def slotRecomputedDocument(self, doc):
      self.signal('DocRecomputed', doc)

    def slotOpenTransaction(self, doc, name):
      self.signal('DocOpenTransaction', doc, name)

    def slotCommitTransaction(self, doc):
      self.signal('DocCommitTransaction', doc)

    def slotAbortTransaction(self, doc):
      self.signal('DocAbortTransaction', doc)

    def slotUndo(self):
      self.signal('Undo')

    def slotRedo(self):
      self.signal('Redo')

    #def slotBeforeCloseTransaction(self, abort):

    #def slotCloseTransaction(self, abort)

    def slotAppendDynamicProperty(self, obj, prop):
      self.signal('ObjAddDynProp', obj, prop)

    def slotRemoveDynamicProperty(self, obj, prop):
      self.signal('ObjRemoveDynProp', obj, prop)

    def slotChangePropertyEditor(self, obj, prop):
      self.signal('ObjChangePropEdit', obj, prop)

    def slotStartSaveDocument(self, obj, name):
      self.signal('DocStartSave', obj, name)

    def slotFinishSaveDocument(self, obj, name):
      self.signal('DocFinishSave', obj, name)

    def slotBeforeAddingDynamicExtension(self, container, extension):
      self.signal('DExtBeforeAdding', container, extension)

    def slotAddedDynamicExtension(self, container, extension):
      self.signal('DExtAdded', container, extension)


class GuiObserver(Observer):
    # Актуализировать можн она github.com FreeCAD/src/Gui/DocumentObserverPython.h
    def __init__(self, ownerClass, isDebug=False):
        self.connectFunc = Gui.addDocumentObserver
        self.disconnectFunc = Gui.removeDocumentObserver
        super().__init__('GuiObserver', ownerClass, isDebug)


    def slotCreatedDocument(self, doc):
      self.signal('DocCreated', doc)

    def slotDeletedDocument(self, doc):
      self.signal('DocDeleted', doc)

    def slotRelabelDocument(self, doc):
      self.signal('DocRelabled', doc)

    def slotRenameDocument(self, doc):
      self.signal('DocRenamed', doc)

    def slotActivateDocument(self, doc):
      self.signal('DocActivated', doc)

    def slotCreatedObject(self, obj):
      self.signal('ObjCreated', obj)

    def slotDeletedObject(self, obj):
      self.signal('ObjDeleted', obj)

    def slotBeforeChangeObject(obj, prop):
      self.signal('ObjBeforeChange', obj, prop)

    def slotChangedObject(self, obj, prop):
      self.signal('ObjChanged', obj, prop)

    def slotInEdit(self, obj):
      self.signal('ObjInEdit', obj)

    def slotResetEdit(self, obj):
      self.signal('ObjResetEdit', obj)
