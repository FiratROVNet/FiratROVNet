from ursina import *
from panda3d.bullet import BulletWorld, BulletRigidBodyNode, BulletBoxShape, BulletPlaneShape
from panda3d.core import Vec3, Quat

# 1. PENCERE AYARLARI
window.forced_aspect_ratio = 1.77
window.size = (1536, 864)
app = Ursina()

# 2. GÖRSEL ORTAM
Sky()
DirectionalLight(y=3, z=3, shadows=True)
grid = Entity(model=Grid(40, 40), scale=40, rotation_x=90, color=color.white33)

# 3. FİZİK DÜNYASI
world = BulletWorld()
world.setGravity(Vec3(0, -9.81, 0))

# --- ROV SİSTEMİ ---
def create_rov_system():
    ent = Entity(model='cube', scale=(2, 1, 3), color=color.gray)
    node = BulletRigidBodyNode('ROV_Physics')
    node.setMass(15.0)
    
    # Damping (Sönümleme) - Değerleri biraz düşürdük ki daha akıcı dönsün
    node.setLinearDamping(0.5) 
    node.setAngularDamping(0.7) # 0.95 çok yüksekti, 0.7 daha iyi
    
    shape = BulletBoxShape(Vec3(1, 0.5, 1.5))
    node.addShape(shape)
    
    np = render.attachNewNode(node)
    np.setPos(0, 5, 0)
    world.attachRigidBody(node)
    return ent, node, np

rov_entity, rov_node, rov_np = create_rov_system()

# Motor Pozisyonları (Lokal sabitler)
M_SOL_POS = Vec3(-1.1, 0, -1.2)
M_SAG_POS = Vec3(1.1, 0, -1.2)

# Görsel Motorlar
motor_sol_vis = Entity(parent=rov_entity, model='cube', scale=(.2, .6, .2), 
                       position=M_SOL_POS, color=color.red, rotation_x=90)
motor_sag_vis = Entity(parent=rov_entity, model='cube', scale=(.2, .6, .2), 
                       position=M_SAG_POS, color=color.red, rotation_x=90)

# ZEMİN
ground_node = BulletRigidBodyNode('Ground')
ground_node.addShape(BulletPlaneShape(Vec3(0, 1, 0), 0))
world.attachRigidBody(ground_node)

EditorCamera()
camera.position = (0, 15, -20)

# --- ANA DÖNGÜ ---
def update():
    dt = time.dt
    world.doPhysics(dt)
    
    # Fizik ve Görsel Senkronizasyon
    rov_entity.position = rov_np.getPos()
    rov_entity.quaternion = rov_np.getQuat()

    # MEVCUT DÖNÜŞ (Quaternion)
    # Bu, lokal koordinatları dünya koordinatlarına çevirmek için şarttır.
    current_quat = rov_np.getQuat()
    forward_vec = current_quat.getForward()

    # MOTORLARIN DÜNYADAKİ GÜNCEL POZİSYONLARI (Relative to Center)
    # .xform() metodu, lokal bir noktayı nesnenin dönüşüne göre döndürür.
    sol_motor_world_pos = current_quat.xform(M_SOL_POS)
    sag_motor_world_pos = current_quat.xform(M_SAG_POS)

    force_power = 800
    turn_power = 10

    # İLERİ / GERİ
    if held_keys['w']:
        rov_node.applyForce(forward_vec * force_power, sol_motor_world_pos)
        rov_node.applyForce(forward_vec * force_power, sag_motor_world_pos)
    
    if held_keys['s']:
        rov_node.applyForce(forward_vec * -force_power, sol_motor_world_pos)
        rov_node.applyForce(forward_vec * -force_power, sag_motor_world_pos)

    # SÜREKLİ DÖNÜŞ (A-D)
    # Artık itki noktaları (world_pos) araçla beraber döndüğü için kilitlenme olmaz.
    if held_keys['a']:
        # Sola dön: Sağ motor ileri, Sol motor geri
        rov_node.applyForce(forward_vec * turn_power, sag_motor_world_pos)
        rov_node.applyForce(forward_vec * -turn_power, sol_motor_world_pos)

    if held_keys['d']:
        # Sağa dön: Sol motor ileri, Sağ motor geri
        rov_node.applyForce(forward_vec * turn_power, sol_motor_world_pos)
        rov_node.applyForce(forward_vec * -turn_power, sag_motor_world_pos)

    # SIFIRLAMA
    if held_keys['r']:
        rov_np.setPos(Vec3(0, 5, 0))
        rov_np.setQuat(Quat())
        rov_node.setLinearVelocity(Vec3(0,0,0))
        rov_node.setAngularVelocity(Vec3(0,0,0))

app.run()