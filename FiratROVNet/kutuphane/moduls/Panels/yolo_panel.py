from panda3d.core import Texture as P3DTexture
from ursina import Entity, Text, camera, color, destroy

try:
    import cv2
    import numpy as np
    CV2_OK = True
except Exception:
    cv2 = None
    np = None
    CV2_OK = False

# Mod renk etiketleri
_MOD_RENK = {
    "renk":   color.rgba(255, 100,  50, 220),
    "model":  color.rgba( 50, 200, 255, 220),
    "hibrit": color.rgba( 80, 255, 120, 220),
}


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
        # Mod göstergesi
        self.mod_etiketi = Text(
            parent=self.root,
            text="MOD: hibrit",
            origin=(0, 0),
            position=(0, -0.60),
            scale=1.8,
            color=_MOD_RENK.get("hibrit"),
        )
        # HSV renk aralığı önizleme şeridi
        self._renk_seridi = None
        self._renk_seridi_tex = None
        self._renk_seridi_olustur()
        self._p3d_tex = None

    def _renk_seridi_olustur(self):
        """Config'deki HSV aralıklarından renkli önizleme şeridi oluşturur."""
        if not CV2_OK:
            return
        try:
            from FiratROVNet.config import TespitAyarlari
            araliklar = list(TespitAyarlari.RENK_ARALIKLAR)
        except Exception:
            return

        # Her renk için orta HSV değerini BGR'ye çevir, 30×30 kare oluştur
        kareler = []
        etiketler = []
        for isim, lower, upper in araliklar:
            if isim == 'kirmizi2':
                continue
            h_mid = (int(lower[0]) + int(upper[0])) // 2
            s_mid = (int(lower[1]) + int(upper[1])) // 2
            v_mid = (int(lower[2]) + int(upper[2])) // 2
            hsv_px = np.uint8([[[h_mid, s_mid, v_mid]]])
            bgr = cv2.cvtColor(hsv_px, cv2.COLOR_HSV2BGR)[0][0]
            kare = np.full((30, 40, 3), bgr.tolist(), dtype=np.uint8)
            # Alt çizgi: lower rengi
            lower_bgr = cv2.cvtColor(np.uint8([[[lower[0], lower[1], lower[2]]]]), cv2.COLOR_HSV2BGR)[0][0]
            upper_bgr = cv2.cvtColor(np.uint8([[[upper[0], upper[1], upper[2]]]]), cv2.COLOR_HSV2BGR)[0][0]
            kare[0:5, :] = lower_bgr
            kare[25:30, :] = upper_bgr
            cv2.putText(kare, isim[:4], (1, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
            kareler.append(kare)

        if not kareler:
            return

        serit = np.hstack(kareler)  # (30, N*40, 3)
        serit_rgb = cv2.cvtColor(serit, cv2.COLOR_BGR2RGB)

        h, w, _ = serit_rgb.shape
        tex = P3DTexture("renk_serit")
        tex.setup2dTexture(w, h, P3DTexture.T_unsigned_byte, P3DTexture.F_rgb)
        tex.setRamImage(serit_rgb[::-1].tobytes())

        self._renk_seridi = Entity(
            parent=self.root,
            model="quad",
            scale=(1.0, 0.18),
            position=(0, -0.78),
            z=-0.01,
            unlit=True,
        )
        self._renk_seridi.model.setTexture(tex, 1)
        self._renk_seridi_tex = tex

    def mod_guncelle(self, mod: str):
        """Aktif tespit modunu panelde gösterir."""
        mod = str(mod).lower()
        if self.mod_etiketi:
            self.mod_etiketi.text = f"MOD: {mod}"
            self.mod_etiketi.color = _MOD_RENK.get(mod, color.white)

    def set_rgb_frame(self, annotated_rgb):
        if not CV2_OK:
            return
        height, width, _ = annotated_rgb.shape
        if self._p3d_tex is None or self._p3d_tex.getXSize() != width:
            self._p3d_tex = P3DTexture("yolo_tex")
            self._p3d_tex.setup2dTexture(width, height, P3DTexture.T_unsigned_byte, P3DTexture.F_rgb)
            self.root.model.setTexture(self._p3d_tex, 1)
        flipped = cv2.flip(annotated_rgb, 0)
        self._p3d_tex.setRamImage(flipped.tobytes())

    def set_visible(self, visible: bool):
        self.root.enabled = bool(visible)
