from ursina import Entity, Text, camera, color


def kisayol_paneli_olustur():
    shortcut_root = Entity(parent=camera.ui, position=(0.8, 0.28, -9))
    shortcut_bg = Entity(parent=shortcut_root, model="quad", scale=(0.21, 0.126), color=color.black, z=0.04)
    shortcut_bg.alpha = 0.48
    for border in _kisayol_paneli_kenarlari(shortcut_root):
        border.alpha = 0.30
    Text(
        parent=shortcut_root,
        text=(
            "<white>M<default> Motor   <white>B<default> PID\n"
            "<white>R<default> ROV     <white>G<default> Grup\n"
            "<white>F<default> Görsel  <white>V<default> REC\n"
            "<white>E<default> SAC     <white>2<default> SAC ROV\n"
            "<white>U<default> Arayüz"
        ),
        position=(0, 0, 0),
        origin=(0, 0),
        scale=0.62,
        color=color.gray,
    )
    return shortcut_root


def _kisayol_paneli_kenarlari(parent):
    border_color = color.azure
    return (
        Entity(parent=parent, model="quad", position=(0, 0.063, -0.04), scale=(0.21, 0.002), color=border_color),
        Entity(parent=parent, model="quad", position=(0, -0.063, -0.04), scale=(0.21, 0.002), color=border_color),
        Entity(parent=parent, model="quad", position=(-0.105, 0, -0.04), scale=(0.002, 0.126), color=border_color),
        Entity(parent=parent, model="quad", position=(0.105, 0, -0.04), scale=(0.002, 0.126), color=border_color),
    )
