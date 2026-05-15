from panda3d.core import Texture as P3DTexture
from ursina import Entity, Text, camera, color

try:
    import cv2
except Exception:
    cv2 = None


class YOLOVisionPanel:
    def __init__(self, rov_id, position=(0.5, 0.39), scale=(0.3, 0.2)):
        self.rov_id = rov_id
        self.root = Entity(
            parent=camera.ui,
            model="quad",
            scale=scale,
            position=position,
            z=0,
            color=color.rgba(255, 255, 255, 130),
            unlit=True,
        )
        self.cerceve = Entity(
            parent=self.root,
            model="quad",
            scale=(1.02, 1.03),
            color=color.rgba(0, 255, 255, 150),
            z=0.01,
        )
        self.baslik = Text(
            parent=self.root,
            text=f"ROV-{rov_id} AI VISION",
            origin=(0, 0),
            position=(0, 0.55),
            scale=2.2,
            color=color.rgba(0, 255, 255, 220),
        )
        self._p3d_tex = None

    def set_rgb_frame(self, annotated_rgb):
        height, width, _ = annotated_rgb.shape
        if self._p3d_tex is None or self._p3d_tex.getXSize() != width:
            self._p3d_tex = P3DTexture("yolo_tex")
            self._p3d_tex.setup2dTexture(width, height, P3DTexture.T_unsigned_byte, P3DTexture.F_rgb)
            self.root.model.setTexture(self._p3d_tex, 1)

        if cv2 is not None:
            annotated_rgb = cv2.flip(annotated_rgb, 0)
        else:
            annotated_rgb = annotated_rgb[::-1]
        self._p3d_tex.setRamImage(annotated_rgb.tobytes())

    def set_visible(self, visible: bool):
        self.root.enabled = bool(visible)
