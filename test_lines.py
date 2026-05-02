from ursina import *
from panda3d.core import GeomLines

app = Ursina()
if 'lines' not in Mesh._modes:
    Mesh._modes['lines'] = GeomLines

m = Mesh(vertices=[(0,0,0), (1,1,0), (2,0,0), (3,1,0)], mode='lines')
m.generate()
print("Success!")
os._exit(0)
