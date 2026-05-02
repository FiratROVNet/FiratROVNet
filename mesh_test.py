from ursina import *
app = Ursina()
m = Mesh(vertices=[(0,0,0), (1,1,0), (2,0,0), (3,1,0)], mode='lines')
print("Mesh mode 'lines' accepted.")
os._exit(0)
