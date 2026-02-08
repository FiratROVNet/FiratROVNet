from ursina import Vec3

def sim_to_ursina(sim_x, sim_y, sim_depth):
    """Simülasyon (X, Y_ileri, Z_derinlik) -> Ursina (X, Y_yukarı, Z_ileri)"""
    # Mapping: Ursina X=SimX | Ursina Y=-SimDepth | Ursina Z=SimY
    return Vec3(float(sim_x), -float(sim_depth), float(sim_y))

def ursina_to_sim(ux, uy, uz):
    """Ursina (X, Y, Z) -> Simülasyon (X, Y_ileri, Z_derinlik)"""
    return (ux, uz, -uy)