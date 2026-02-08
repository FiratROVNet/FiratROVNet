from ursina import *
from math import sin, cos, atan2, degrees, pi
import random

# --- UYGULAMA ve PENCERE AYARLARI ---
window.size = (1536, 864)
app = Ursina(borderless=False)

# --- ORTAM SABİTLERİ ---
OCEAN_XZ_SCALE = 1000
OCEAN_DEPTH = 100
SEA_FLOOR_Y = -OCEAN_DEPTH / 2
WATER_SURFACE_Y_BASE = OCEAN_DEPTH / 2
OCEAN_BOUNDARY_XZ = OCEAN_XZ_SCALE * 0.48
FISH_MIN_CLEARANCE = 2.0

# --- ORTAM NESNELERİ ---
Entity(
    model="./assets/water/my_models/ocean_taban/sand_envi_034.fbx",
    scale=(2.2 * (OCEAN_XZ_SCALE / 500), 1, 2 * (OCEAN_XZ_SCALE / 500)),
    position=(0, SEA_FLOOR_Y, 0),
    texture="./assets/water/my_models/ocean_taban/sand_envi_034-0.jpg",
    double_sided=True,
    collider='mesh'
)
Entity(
    model="cube",
    scale=(OCEAN_XZ_SCALE, OCEAN_DEPTH, OCEAN_XZ_SCALE),
    position=(0, 0, 0),
    color=color.rgba(0.2, 0.4, 0.8, 0.05), # Su hacmi rengi
    double_sided=True,
    render_queue=1
)
ocean_surface = Entity(
    model="plane",
    scale=(OCEAN_XZ_SCALE, 1, OCEAN_XZ_SCALE),
    position=(0, WATER_SURFACE_Y_BASE, 0),
    texture="./assets/water/my_models/water4.jpg", # Ana renk dokusu
    texture_scale=(1, 1), # Ana doku tekrar etmiyor
    normals=Texture('map/water4_normal.png'), # NORMAL MAP ATANDI
    # Normal map'in ne kadar tekrar edeceğini ve ne kadar güçlü olacağını ayarlayabilirsiniz
    # normal_scale, normal map'in UV'lerini ölçekler (daha küçük değerler -> daha büyük desenler)
    # shader_input ile normal map'in gücünü de ayarlayabilirsiniz (varsayılan shader destekliyorsa)
    # Şimdilik varsayılan ölçek ve güçte bırakalım.
    # Eğer normal map'in kendisi tekrarlanabilir (tileable) ise, texture_scale'den farklı bir
    # normal_scale kullanabilirsiniz, örn: ocean_surface.set_shader_input('normal_scale', Vec2(10,10))
    # Ancak Ursina'da normal_scale doğrudan Entity parametresi değil, shader_input ile ayarlanır.
    # En basiti, normal map'in kendisini tileable yapıp texture_offset ile kaydırmak.
    # Veya normal map'in de texture_scale'ini (1,1) bırakıp,
    # normal map'in kendisi küçük, tekrarlayan desenler içeriyorsa,
    # bu da işe yarayabilir.
    # Eğer normal map'in UV'lerini ana dokudan bağımsız ölçeklemek isterseniz, custom shader gerekebilir.
    # ŞİMDİLİK, normal map'in de ana doku gibi (1,1) ölçeklendiğini varsayalım
    # ve sadece offset'ini değiştirelim.
    double_sided=True,
    color=color.rgba(0.4, 0.7, 0.9, 0.75) # Alpha değeri ayarlandı
)

# --- DENİZALTI ---
SUBMARINE_SCALE_FACTOR = 15
sub_marine = Entity(
    model="./assets/water/my_models/submarine.obj",
    scale=SUBMARINE_SCALE_FACTOR,
    position=(10, WATER_SURFACE_Y_BASE - 20, 20),
    rotation=(0, 45, 0)
)

# --- BALIK OLUŞTURMA ---
FISH_BASE_SCALE = 0.18
fish_list = []
fish_half_height_model_space = 0.15
NUMBER_OF_FISH_TO_CREATE = 15

def create_fish(x, z, y_rotation, scale_multiplier=1.0, name="fish"):
    current_fish_scale = FISH_BASE_SCALE * scale_multiplier
    actual_half_height = current_fish_scale * fish_half_height_model_space
    min_y_start = SEA_FLOOR_Y + FISH_MIN_CLEARANCE + actual_half_height
    max_y_start = WATER_SURFACE_Y_BASE - FISH_MIN_CLEARANCE - actual_half_height
    fish_y = random.uniform(min_y_start, max_y_start if max_y_start > min_y_start else min_y_start)
    fish_entity = Entity(
        name=name, model='./assets/water/FBX/TroutRainbow.fbx', scale=current_fish_scale,
        position=(x, fish_y, z), rotation=(0, y_rotation, 0),
        texture="./assets/water/FBX/TroutRainbow.fbm/TroutRainbow_baseColor.jpg",
        move_pattern_x = None, move_pattern_y = None, move_pattern_z = None,
        move_amplitude = Vec3(random.uniform(15, OCEAN_BOUNDARY_XZ * 0.5), # Genlikleri biraz artırdım
                              random.uniform(0.8, 2.5),
                              random.uniform(15, OCEAN_BOUNDARY_XZ * 0.5)),
        move_frequency = Vec3(random.uniform(0.04, 0.12),
                              random.uniform(0.25, 0.85),
                              random.uniform(0.04, 0.12)),
        base_y_pos = fish_y, initial_pos = Vec3(x, fish_y, z), actual_half_height = actual_half_height,
        selected_move_pattern_x = random.choice(["sin", "cos", None]),
        selected_move_pattern_y = random.choice(["sin", "cos", "sin_relative_to_wave", None]),
        selected_move_pattern_z = random.choice(["sin", "cos", None])
    )
    fish_entity.move_pattern_x = fish_entity.selected_move_pattern_x
    fish_entity.move_pattern_y = fish_entity.selected_move_pattern_y
    fish_entity.move_pattern_z = fish_entity.selected_move_pattern_z
    if not fish_entity.move_pattern_x and not fish_entity.move_pattern_z:
        if random.random() < 0.5: fish_entity.move_pattern_x = "sin"
        else: fish_entity.move_pattern_z = "cos"
    fish_list.append(fish_entity)
    return fish_entity

for i in range(NUMBER_OF_FISH_TO_CREATE):
    rand_x_start = random.uniform(-OCEAN_BOUNDARY_XZ * 0.7, OCEAN_BOUNDARY_XZ * 0.7)
    rand_z_start = random.uniform(-OCEAN_BOUNDARY_XZ * 0.7, OCEAN_BOUNDARY_XZ * 0.7)
    rand_y_rot_start = random.uniform(0, 360)
    rand_scale_mult = random.uniform(0.85, 1.15)
    create_fish(x=rand_x_start, z=rand_z_start, y_rotation=rand_y_rot_start,
                scale_multiplier=rand_scale_mult, name=f"fish_{i}")

# --- DALGA HAREKETİ ve GÜNCELLEME ---
simulation_time = 0
WAVE_AMPLITUDE = 1.5 # Yüzeyin dikey genliğini biraz daha artırdım
WAVE_FREQUENCY = 0.8
current_wave_y_surface = WATER_SURFACE_Y_BASE

normal_map_offset_u = 0 # Normal map X offset (U koordinatı)
normal_map_offset_v = 0 # Normal map Y offset (V koordinatı)
NORMAL_MAP_SCROLL_SPEED_U = 0.02 # Normal map'in U yönünde kayma hızı
NORMAL_MAP_SCROLL_SPEED_V = 0.01 # Normal map'in V yönünde kayma hızı

def apply_movement(entity, sim_time):
    # ... (apply_movement fonksiyonunun içeriği öncekiyle aynı, değişiklik yok) ...
    new_pos_offset = Vec3(0,0,0)
    if entity.move_pattern_x == "sin": new_pos_offset.x = sin(sim_time * entity.move_frequency.x) * entity.move_amplitude.x
    elif entity.move_pattern_x == "cos": new_pos_offset.x = cos(sim_time * entity.move_frequency.x) * entity.move_amplitude.x
    target_y_offset = 0
    if entity.move_pattern_y == "sin": target_y_offset = sin(sim_time * entity.move_frequency.y) * entity.move_amplitude.y
    elif entity.move_pattern_y == "cos": target_y_offset = cos(sim_time * entity.move_frequency.y) * entity.move_amplitude.y
    elif entity.move_pattern_y == "sin_relative_to_wave":
        relative_depth = WATER_SURFACE_Y_BASE - entity.base_y_pos
        target_y_offset = (current_wave_y_surface - relative_depth) - entity.base_y_pos + \
                           sin(sim_time * entity.move_frequency.y) * entity.move_amplitude.y
    new_pos_offset.y = target_y_offset
    if entity.move_pattern_z == "sin": new_pos_offset.z = sin(sim_time * entity.move_frequency.z) * entity.move_amplitude.z
    elif entity.move_pattern_z == "cos": new_pos_offset.z = cos(sim_time * entity.move_frequency.z) * entity.move_amplitude.z
    calculated_pos = Vec3(entity.initial_pos.x + new_pos_offset.x,
                          entity.base_y_pos + new_pos_offset.y,
                          entity.initial_pos.z + new_pos_offset.z)
    fish_hh = entity.actual_half_height
    min_y = SEA_FLOOR_Y + FISH_MIN_CLEARANCE + fish_hh
    max_y = current_wave_y_surface - FISH_MIN_CLEARANCE - fish_hh
    calculated_pos.y = clamp(calculated_pos.y, min_y if min_y < max_y else max_y - 0.1, max_y if max_y > min_y else min_y + 0.1)
    calculated_pos.x = clamp(calculated_pos.x, -OCEAN_BOUNDARY_XZ, OCEAN_BOUNDARY_XZ)
    calculated_pos.z = clamp(calculated_pos.z, -OCEAN_BOUNDARY_XZ, OCEAN_BOUNDARY_XZ)
    direction_of_movement = calculated_pos - entity.world_position
    entity.position = calculated_pos
    if direction_of_movement.length_squared() > 0.0001:
        angle_y = degrees(atan2(direction_of_movement.x, direction_of_movement.z))
        entity.rotation_y = lerp_angle(entity.rotation_y, angle_y, time.dt * 2)


def update():
    global simulation_time, current_wave_y_surface
    global normal_map_offset_u, normal_map_offset_v # Global olarak eriş
    simulation_time += time.dt

    # Okyanus yüzeyi dikey hareketi
    current_wave_y_surface = WATER_SURFACE_Y_BASE + sin(simulation_time * WAVE_FREQUENCY) * WAVE_AMPLITUDE
    ocean_surface.y = current_wave_y_surface

    # Normal Map Hareketi
    if hasattr(ocean_surface, 'normals') and ocean_surface.normals:
        normal_map_offset_u += time.dt * NORMAL_MAP_SCROLL_SPEED_U
        normal_map_offset_v += time.dt * NORMAL_MAP_SCROLL_SPEED_V
        # Offset değerleri 0-1 aralığında kalmalı, gerekirse % 1.0
        ocean_surface.texture_offset = (normal_map_offset_u % 1.0, normal_map_offset_v % 1.0)

    # Denizaltı dalga etkileşimi
    sub_marine_y_offset = sin(simulation_time * WAVE_FREQUENCY * 0.5 + 1) * (WAVE_AMPLITUDE * 0.2)
    sub_marine.y = WATER_SURFACE_Y_BASE - 20 + sub_marine_y_offset

    # Balıkların hareketi
    for fish_entity in fish_list:
        apply_movement(fish_entity, simulation_time)

# --- KAMERA ve IŞIK ---
Sky(color=color.sky)
EditorCamera(position=(0, WATER_SURFACE_Y_BASE + 25, - (OCEAN_XZ_SCALE / 10) ), rotation_x=15)
sun = DirectionalLight(y=15, z=20, shadows=False, rotation=(30, -30, 0))
AmbientLight(color=color.rgba(150,150,180,255))

app.run()
