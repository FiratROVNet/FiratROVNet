from time import monotonic
from typing import Any, cast

from ursina import Entity, Mesh, Text, Vec3, camera, color, destroy, held_keys, mouse  # type: ignore[import]


class APFGucHUD:
    """Tek ROV icin APF guc gecmisi paneli."""

    HISTORY_LIMIT = 150

    CHANNELS = (
        ("hedef_guc", "HEDEF_GUCU", color.lime),
        ("engel_guc", "ENGEL_GUCU", color.red),
        ("rov_guc", "ROV_GUCU", color.orange),
    )

    def _tus_basili(self, key: str) -> bool:
        try:
            return bool(cast(Any, held_keys).get(key, 0))
        except Exception:
            return False

    def _normal_vec3_al(self, vector):
        try:
            normalized = Vec3(
                float(getattr(vector, "x", 0.0)),
                float(getattr(vector, "y", 0.0)),
                float(getattr(vector, "z", 0.0)),
            ).normalized()
        except Exception:
            return None
        return normalized if normalized is not None else None

    def _vec3_bilesenleri_al(self, vector):
        if vector is None or isinstance(vector, bool):
            return None
        try:
            return float(getattr(vector, "x")), float(getattr(vector, "y")), float(getattr(vector, "z"))
        except Exception:
            return None

    def _rov_id_al(self, rov, varsayilan: int = -1) -> int:
        try:
            return int(getattr(rov, "id", varsayilan))
        except Exception:
            return varsayilan

    def _aktif_menu_rov_id_al(self):
        if self._menu_rov_id is None:
            return None
        try:
            return int(self._menu_rov_id)
        except Exception:
            return None

    def _int_deger_al(self, value, varsayilan: int = 0) -> int:
        try:
            if value is None:
                return varsayilan
            return int(value)
        except Exception:
            return varsayilan

    def __init__(self, filo_ref, position=(0.42, 0.30), visible=False):
        self.filo = filo_ref
        self.visible = bool(visible)
        self.position = position
        self.rov_ids = None
        self.selected_index = 0
        self._toggle_down = False
        self._next_down = False
        self._mouse_down = False
        self._enter_down = False
        self._up_down = False
        self._down_down = False
        self._esc_down = False
        self._depth_key_down = {}
        self._menu_root = None
        self._menu_rov_id = None
        self._menu_items = []
        self._menu_index = 0
        self._menu_mod = "main"
        self._depth_text = ""
        self._pending_action = None
        self._rov_marker = None
        self._rov_marker_id = None
        self._minimap_marker = None
        self._minimap_marker_id = None
        self._gecici_marker_bitis = 0.0

        self.root = Entity(parent=camera.ui, position=(position[0], position[1], -8), enabled=self.visible)
        self.rows = []
        self._build()

    def _build(self):
        self.panel = Entity(
            parent=self.root,
            model="quad",
            scale=(0.36, 0.42),
            color=color.black,
            z=0.10,
        )
        self.panel.alpha = 0.46

        header = Entity(
            parent=self.root,
            model="quad",
            position=(0, 0.177, -0.01),
            scale=(0.348, 0.046),
            color=color.black,
        )
        header.alpha = 0.74

        accent = Entity(
            parent=self.root,
            model="quad",
            position=(-0.164, 0.177, -0.03),
            scale=(0.006, 0.031),
            color=color.lime,
        )
        accent.alpha = 1.0

        border_color = color.azure
        for x, y, sx, sy in (
            (0, 0.210, 0.36, 0.002),
            (0, -0.210, 0.36, 0.002),
            (-0.180, 0, 0.002, 0.42),
            (0.180, 0, 0.002, 0.42),
        ):
            border = Entity(parent=self.root, model="quad", position=(x, y, -0.02), scale=(sx, sy), color=border_color)
            border.alpha = 0.72

        self.title = Text(
            parent=self.root,
            text="ROV-0 APF POWER",
            position=(-0.150, 0.176, -0.03),
            origin=(-0.5, 0),
            scale=0.76,
            color=color.white,
        )

        self.hint = Text(
            parent=self.root,
            text="1 TOGGLE   2 NEXT",
            position=(0.148, 0.186, -0.03),
            origin=(0.5, 0),
            scale=0.38,
            color=color.white,
        )
        self.hint.alpha = 0.92

        graph_width = 0.265
        graph_height = 0.070
        row_step = 0.112
        first_y = 0.080
        for row_i, (attr, label_text, line_color) in enumerate(self.CHANNELS):
            center_y = first_y - row_i * row_step
            label = Text(
                parent=self.root,
                text=label_text,
                position=(-0.158, center_y + 0.033, -0.03),
                origin=(-0.5, 0),
                scale=0.54,
                color=line_color,
            )
            label.alpha = 1.0
            value = Text(
                parent=self.root,
                text="0.00",
                position=(0.138, center_y + 0.033, -0.03),
                origin=(0.5, 0),
                scale=0.52,
                color=line_color,
            )
            value.alpha = 1.0

            track = Entity(
                parent=self.root,
                model="quad",
                position=(0.010, center_y - 0.012, -0.01),
                scale=(graph_width, graph_height),
                color=color.black,
            )
            track.alpha = 0.66

            row_accent = Entity(
                parent=self.root,
                model="quad",
                position=(-0.127, center_y - 0.012, -0.03),
                scale=(0.003, graph_height),
                color=line_color,
            )
            row_accent.alpha = 1.0

            baseline = Entity(
                parent=self.root,
                model="quad",
                position=(0.010, center_y - 0.047, -0.03),
                scale=(graph_width, 0.0015),
                color=color.white,
            )
            baseline.alpha = 0.42

            grid_lines = []
            for grid_y in (center_y - 0.012, center_y + 0.023):
                grid = Entity(
                    parent=self.root,
                    model="quad",
                    position=(0.010, grid_y, -0.025),
                    scale=(graph_width, 0.001),
                    color=color.gray,
                )
                grid.alpha = 0.34
                grid_lines.append(grid)

            mesh = Mesh(vertices=[Vec3(0, 0, 0), Vec3(0.001, 0, 0)], mode="line", thickness=3, static=False)
            line = Entity(parent=self.root, model=mesh, color=line_color, z=-0.05, enabled=False)

            self.rows.append(
                {
                    "attr": attr,
                    "label": label,
                    "value": value,
                    "track": track,
                    "row_accent": row_accent,
                    "baseline": baseline,
                    "grid_lines": grid_lines,
                    "line": line,
                    "x0": 0.010 - graph_width / 2.0,
                    "y0": center_y - 0.012 - graph_height / 2.0,
                    "width": graph_width,
                    "height": graph_height,
                }
            )

    def set_rov_ids(self, rov_ids):
        if rov_ids is None:
            self.rov_ids = None
        else:
            self.rov_ids = [int(i) for i in rov_ids]
        self.selected_index = 0

    def set_visible(self, visible: bool):
        self.visible = bool(visible)
        self.root.enabled = self.visible
        if not self.visible:
            rov = self._aktif_marker_rov_al()
            if rov is not None and (self._menu_root is not None or self._pending_action is not None):
                self._secim_markerlarini_guncelle(rov)
            else:
                self._secim_markerlarini_temizle()
        else:
            self._gecici_marker_bitis = 0.0
            self._pending_action = None

    def toggle(self):
        self.set_visible(not self.visible)

    def update(self, process_input: bool = True, draw: bool = True):
        if process_input:
            self._klavye_kontrol()
            self._fare_secim_kontrol()
            self._gecici_marker_suresini_kontrol()
            self._menu_transform_guncelle()
            self._menu_gorsel_guncelle()
        if not draw or not self.visible:
            return

        rov = self._aktif_rov_al()
        if rov is None:
            self.title.text = "ROV-- APF POWER"
            self._rows_enabled(False)
            self._rov_marker_temizle()
            self._minimap_marker_temizle()
            return

        self._rows_enabled(True)
        self._panel_guncelle(rov)
        self._secim_markerlarini_guncelle(rov)

    def _klavye_kontrol(self):
        toggle_down = self._tus_basili("1") or self._tus_basili("num 1")
        next_down = self._tus_basili("2") or self._tus_basili("num 2")
        enter_down = self._tus_basili("enter") or self._tus_basili("return")
        up_down = self._tus_basili("up arrow") or self._tus_basili("w")
        down_down = self._tus_basili("down arrow") or self._tus_basili("s")
        esc_down = self._tus_basili("escape")

        if toggle_down and not self._toggle_down:
            self.toggle()
        self._toggle_down = toggle_down

        if self.visible and next_down and not self._next_down:
            self._sonraki_rov()
        self._next_down = next_down

        if self._menu_root is not None:
            if esc_down and not self._esc_down:
                self._menu_kapat()
            if self._menu_mod == "depth":
                self._depth_klavye_guncelle(enter_down)
            else:
                if up_down and not self._up_down:
                    self._menu_index = (self._menu_index - 1) % max(1, len(self._menu_items))
                if down_down and not self._down_down:
                    self._menu_index = (self._menu_index + 1) % max(1, len(self._menu_items))
                if enter_down and not self._enter_down:
                    self._menu_item_calistir(self._menu_index)
        self._enter_down = enter_down
        self._up_down = up_down
        self._down_down = down_down
        self._esc_down = esc_down

    def _fare_secim_kontrol(self):
        try:
            down = bool(getattr(mouse, "left", False))
        except Exception:
            down = False

        if down and not self._mouse_down:
            if self._pending_hedef_tiklamasi_isle():
                self._mouse_down = down
                return
            if self._menu_tiklamasi_isle():
                self._mouse_down = down
                return
            rov = self._tiklanan_rov_al()
            if rov is not None:
                rov_id = self._rov_id_al(rov)
                ayni_rov_secili = self._rov_marker_id == rov_id
                self._rov_sec(rov)
                self._secim_markerlarini_guncelle(rov)
                if ayni_rov_secili:
                    self._menu_ac(rov)
                    self._gecici_marker_bitis = 0.0
                else:
                    self._menu_kapat()
                    self._gecici_marker_bitis = 0.0 if self.visible else monotonic() + 5.0
            elif self._menu_root is not None:
                self._menu_kapat()
                if self._rov_marker_id is not None:
                    self._gecici_marker_bitis = 0.0 if self.visible else monotonic() + 5.0
        self._mouse_down = down

    def _tiklanan_rov_al(self):
        entity = getattr(mouse, "hovered_entity", None)
        if entity is not None and self._entity_panel_icinde_mi(entity):
            return None

        rovs = getattr(self.filo, "rovs", []) or []
        if entity is not None:
            current = entity
            while current is not None:
                for rov in rovs:
                    if current is rov:
                        return rov
                current = getattr(current, "parent", None)
        return self._mouse_ray_yakin_rov_al(rovs)

    def _mouse_ray_yakin_rov_al(self, rovs):
        origin = getattr(camera, "world_position", None)
        direction = getattr(mouse, "ray", None)
        if origin is None:
            return None
        if direction is None:
            direction = getattr(camera, "forward", None)
        if direction is None:
            return None
        try:
            origin_xyz = self._vec3_bilesenleri_al(origin)
            if origin_xyz is None:
                return None
            ray_origin = Vec3(origin_xyz[0], origin_xyz[1], origin_xyz[2])
            ray_dir = self._normal_vec3_al(direction)
        except Exception:
            return None
        if ray_dir is None:
            return None

        en_iyi = None
        en_iyi_skor = None
        for rov in rovs:
            try:
                pos = getattr(rov, "world_position", None)
                if pos is None:
                    pos = getattr(rov, "position", None)
                if pos is None:
                    continue
                pos_xyz = self._vec3_bilesenleri_al(pos)
                if pos_xyz is None:
                    continue
                rov_pos = Vec3(pos_xyz[0], pos_xyz[1], pos_xyz[2])
            except Exception:
                continue
            delta = rov_pos - ray_origin
            ileri_mesafe = (
                float(getattr(delta, "x", 0.0)) * float(getattr(ray_dir, "x", 0.0))
                + float(getattr(delta, "y", 0.0)) * float(getattr(ray_dir, "y", 0.0))
                + float(getattr(delta, "z", 0.0)) * float(getattr(ray_dir, "z", 0.0))
            )
            if ileri_mesafe <= 0.0:
                continue
            en_yakin_nokta = ray_origin + (ray_dir * ileri_mesafe)
            ray_mesafe = (rov_pos - en_yakin_nokta).length()
            secim_toleransi = max(2.8, min(18.0, ileri_mesafe * 0.035))
            if ray_mesafe > secim_toleransi:
                continue
            skor = (ray_mesafe / secim_toleransi) + (ileri_mesafe * 0.0005)
            if en_iyi_skor is None or skor < en_iyi_skor:
                en_iyi = rov
                en_iyi_skor = skor
        return en_iyi

    def _entity_panel_icinde_mi(self, entity) -> bool:
        current = entity
        while current is not None:
            if current is self.root:
                return True
            if self._menu_root is not None and current is self._menu_root:
                return True
            current = getattr(current, "parent", None)
        return False

    def _rov_sec(self, rov) -> bool:
        rov_id = self._rov_id_al(rov)
        rovs = self._gosterilecek_rovleri_al()
        for i, aday in enumerate(rovs):
            if self._rov_id_al(aday, -2) == rov_id:
                self.selected_index = i
                return True
        return False

    def _menu_ac(self, rov, mod: str = "main"):
        self._menu_kapat()
        self._menu_rov_id = self._rov_id_al(rov)
        self._menu_mod = mod
        self._menu_index = 0
        self._menu_root = Entity(parent=camera.ui, position=(0, 0, -9), enabled=True)
        self._menu_icerigini_kur(rov)
        self._menu_transform_guncelle()
        self._menu_gorsel_guncelle()

    def _menu_kapat(self):
        if self._menu_root is not None:
            try:
                destroy(self._menu_root)
            except Exception:
                self._menu_root.enabled = False
        self._menu_root = None
        self._menu_rov_id = None
        self._menu_items = []
        self._menu_index = 0
        self._menu_mod = "main"
        self._depth_text = ""

    def _menu_icerigini_kur(self, rov):
        if self._menu_root is None:
            return
        for child in list(getattr(self._menu_root, "children", [])):
            destroy(child)
        if self._menu_mod == "more":
            tanimlar = self._more_menu_tanimlari(rov)
        elif self._menu_mod == "depth":
            tanimlar = [("DEPTH: _", "depth_input"), ("ENTER", "depth_confirm"), ("BACK", "back")]
        else:
            tanimlar = [("GIT", "git"), ("GIT_PATH", "git_path"), ("DEPTH", "depth"), ("MORE", "more")]
        self._menu_items = []

        row_step = 0.064
        row_height = 0.050
        height = 0.084 + (row_step * len(tanimlar))
        width = 0.310
        panel = Entity(
            parent=self._menu_root,
            model="quad",
            scale=(width, height),
            color=color.black,
            z=0.10,
        )
        panel.alpha = 0.58
        header = Entity(
            parent=self._menu_root,
            model="quad",
            position=(0, height / 2.0 - 0.035, -0.01),
            scale=(width - 0.020, 0.052),
            color=color.rgba(10, 24, 34, 235),
        )
        header.alpha = 0.92
        accent = Entity(
            parent=self._menu_root,
            model="quad",
            position=(-width / 2.0 + 0.013, height / 2.0 - 0.035, -0.03),
            scale=(0.004, 0.036),
            color=color.azure,
        )
        accent.alpha = 1.0
        for x, y, sx, sy in (
            (0, height / 2.0, width, 0.002),
            (0, -height / 2.0, width, 0.002),
            (-width / 2.0, 0, 0.002, height),
            (width / 2.0, 0, 0.002, height),
        ):
            border = Entity(
                parent=self._menu_root,
                model="quad",
                position=(x, y, -0.025),
                scale=(sx, sy),
                color=color.azure,
            )
            border.alpha = 0.50
        rov_id = self._rov_id_al(rov)
        title_shadow = Text(
            parent=self._menu_root,
            text=f"ROV-{rov_id}",
            position=(-0.133, height / 2.0 - 0.050, -0.035),
            origin=(-0.5, 0),
            scale=0.74,
            color=color.black,
        )
        title_shadow.alpha = 0.78
        title = Text(
            parent=self._menu_root,
            text=f"ROV-{rov_id}",
            position=(-0.135, height / 2.0 - 0.047, -0.04),
            origin=(-0.5, 0),
            scale=0.74,
            color=color.azure,
        )
        title.alpha = 1.0

        y = height / 2.0 - 0.106
        for idx, (label, action) in enumerate(tanimlar):
            row_y = y - idx * row_step
            text_color, active_bg, idle_bg = self._menu_renklerini_al(action, label)
            row = Entity(
                parent=self._menu_root,
                model="quad",
                position=(0, row_y, -0.02),
                scale=(width - 0.026, row_height),
                color=idle_bg,
                collider="box",
            )
            row._apf_menu_index = idx
            row_accent = Entity(
                parent=self._menu_root,
                model="quad",
                position=(-width / 2.0 + 0.022, row_y, -0.04),
                scale=(0.003, row_height * 0.56),
                color=text_color,
            )
            row_accent.alpha = 0.35
            shadow = Text(
                parent=self._menu_root,
                text=label,
                position=(-0.127, row_y - 0.018, -0.045),
                origin=(-0.5, 0),
                scale=0.68,
                color=color.black,
            )
            shadow.alpha = 0.86
            text = Text(
                parent=self._menu_root,
                text=label,
                position=(-0.130, row_y - 0.016, -0.055),
                origin=(-0.5, 0),
                scale=0.68,
                color=text_color,
            )
            text.alpha = 1.0
            self._text_rengini_uygula(text, text_color)
            self._menu_items.append(
                {
                    "row": row,
                    "accent": row_accent,
                    "shadow": shadow,
                    "text": text,
                    "base_color": text_color,
                    "active_bg": active_bg,
                    "idle_bg": idle_bg,
                    "action": action,
                    "label": label,
                }
            )

    def _menu_renklerini_al(self, action: str, label: str):
        if action in ("git", "depth_confirm"):
            return color.rgb(40, 255, 105), color.rgba(40, 220, 100, 245), color.rgba(16, 56, 32, 205)
        if action == "git_path":
            return color.rgb(0, 224, 255), color.rgba(0, 200, 232, 245), color.rgba(12, 50, 60, 205)
        if action in ("depth", "depth_input"):
            return color.rgb(86, 145, 255), color.rgba(74, 132, 245, 245), color.rgba(18, 38, 76, 205)
        if action in ("more", "toggle_mod"):
            return color.rgb(255, 178, 34), color.rgba(240, 156, 24, 245), color.rgba(72, 46, 12, 205)
        if action == "toggle_manual":
            if "ON" in label:
                return color.rgb(54, 244, 78), color.rgba(54, 216, 72, 245), color.rgba(20, 58, 24, 205)
            return color.rgb(255, 96, 62), color.rgba(238, 88, 52, 245), color.rgba(76, 30, 22, 205)
        if action == "toggle_gps":
            return color.rgb(0, 232, 174), color.rgba(0, 204, 160, 245), color.rgba(10, 58, 48, 205)
        if action == "back":
            return color.rgb(150, 176, 198), color.rgba(126, 152, 176, 245), color.rgba(34, 44, 54, 205)
        return color.rgb(0, 224, 236), color.rgba(0, 200, 212, 245), color.rgba(12, 52, 58, 205)

    def _text_rengini_uygula(self, text_obj, renk):
        try:
            text_obj.color = renk
        except Exception:
            pass
        try:
            text_obj.text_entity.color = renk
        except Exception:
            pass
        try:
            text_obj.alpha = 1.0
        except Exception:
            pass

    def _more_menu_tanimlari(self, rov):
        gnc = getattr(rov, "gnc", None)
        mod = getattr(gnc, "mod", "-")
        manuel = bool(getattr(gnc, "manuel_kontrol", False))
        gps = getattr(gnc, "gps_sinyal", "-")
        return [
            (f"MOD: {mod}", "toggle_mod"),
            (f"MANUEL: {'ON' if manuel else 'OFF'}", "toggle_manual"),
            (f"GPS: {gps}", "toggle_gps"),
            ("BACK", "back"),
        ]

    def _menu_transform_guncelle(self):
        menu_rov_id = self._aktif_menu_rov_id_al()
        if self._menu_root is None or menu_rov_id is None:
            return
        try:
            rov = self.filo.find_rov_by_id(menu_rov_id)
        except Exception:
            rov = None
        if rov is None:
            self._menu_kapat()
            return
        self._menu_root.position = self._rov_ui_konumu_al(rov)

    def _rov_ui_konumu_al(self, rov):
        try:
            pos = getattr(rov, "world_position", None)
            if pos is None:
                pos = getattr(rov, "position", Vec3(0, 0, 0))
            pos_xyz = self._vec3_bilesenleri_al(pos)
            if pos_xyz is None:
                return (0.12, 0.08, -9)
            screen = camera.world_to_screen_point(Vec3(pos_xyz[0], pos_xyz[1] + 2.5, pos_xyz[2]))
            screen_xy = self._vec3_bilesenleri_al(screen)
            if screen_xy is None:
                return (0.12, 0.08, -9)
            x, y, _ = screen_xy
            if 0.0 <= x <= 1.0 and 0.0 <= y <= 1.0:
                x -= 0.5
                y -= 0.5
            return (
                max(-0.72, min(0.72, x + 0.12)),
                max(-0.42, min(0.45, y + 0.04)),
                -9,
            )
        except Exception:
            return (0.12, 0.08, -9)

    def _menu_gorsel_guncelle(self):
        if self._menu_root is None:
            return
        hovered = getattr(mouse, "hovered_entity", None)
        for idx, item in enumerate(self._menu_items):
            row = item["row"]
            current = hovered
            hover = False
            while current is not None:
                if current is row:
                    hover = True
                    break
                current = getattr(current, "parent", None)
            if hover:
                self._menu_index = idx
            secili = idx == self._menu_index
            if secili:
                row.color = item["active_bg"]
                row.alpha = 0.96
                item["accent"].color = color.rgba(8, 18, 24, 255)
                item["accent"].alpha = 0.70
                item["shadow"].alpha = 0.0
                self._text_rengini_uygula(item["text"], color.rgba(4, 12, 16, 255))
            else:
                row.color = item["idle_bg"]
                row.alpha = 0.72
                item["accent"].color = item["base_color"]
                item["accent"].alpha = 0.78
                item["shadow"].alpha = 0.86
                self._text_rengini_uygula(item["text"], item["base_color"])

    def _menu_tiklamasi_isle(self) -> bool:
        if self._menu_root is None:
            return False
        entity = getattr(mouse, "hovered_entity", None)
        current = entity
        while current is not None:
            idx = getattr(current, "_apf_menu_index", None)
            if idx is not None:
                self._menu_item_calistir(int(idx))
                return True
            if current is self._menu_root:
                return True
            current = getattr(current, "parent", None)
        return False

    def _menu_item_calistir(self, index: int):
        if not self._menu_items:
            return
        index = max(0, min(index, len(self._menu_items) - 1))
        action = self._menu_items[index]["action"]
        menu_rov_id = self._aktif_menu_rov_id_al()
        if menu_rov_id is None:
            self._menu_kapat()
            return
        try:
            rov = self.filo.find_rov_by_id(menu_rov_id)
        except Exception:
            rov = None
        if rov is None:
            self._menu_kapat()
            return

        if action in ("git", "git_path"):
            self._pending_action = {"type": action, "rov_id": self._rov_id_al(rov)}
            self._menu_kapat()
            self._secim_markerlarini_guncelle(rov)
            self._gecici_marker_bitis = 0.0
        elif action == "depth":
            self._depth_text = ""
            self._menu_mod = "depth"
            self._menu_icerigini_kur(rov)
        elif action == "depth_confirm":
            self._depth_komutunu_calistir(rov)
        elif action == "more":
            self._menu_mod = "more"
            self._menu_icerigini_kur(rov)
        elif action == "back":
            self._menu_mod = "main"
            self._menu_icerigini_kur(rov)
        elif action == "toggle_mod":
            gnc = getattr(rov, "gnc", None)
            if gnc is not None:
                gnc.mod = 0 if self._int_deger_al(getattr(gnc, "mod", 1), 1) != 0 else 1
            self._menu_icerigini_kur(rov)
        elif action == "toggle_manual":
            gnc = getattr(rov, "gnc", None)
            if gnc is not None:
                gnc.manuel_kontrol = not bool(getattr(gnc, "manuel_kontrol", False))
            self._menu_icerigini_kur(rov)
        elif action == "toggle_gps":
            gnc = getattr(rov, "gnc", None)
            if gnc is not None:
                gnc.gps_sinyal = 0 if self._int_deger_al(getattr(gnc, "gps_sinyal", 1), 1) else 1
            self._menu_icerigini_kur(rov)
        self._menu_gorsel_guncelle()

    def _pending_hedef_tiklamasi_isle(self) -> bool:
        if self._pending_action is None:
            return False
        entity = getattr(mouse, "hovered_entity", None)
        if entity is not None and self._entity_panel_icinde_mi(entity):
            return True
        hedef = self._tiklanan_hedef_noktasi_al()
        if hedef is None:
            return True
        action = cast(dict[str, Any], self._pending_action)
        self._pending_action = None
        rov_id = self._int_deger_al(action.get("rov_id"), -1)
        try:
            if action.get("type") == "git_path":
                self.filo.git_path(rov_id, hedef, ai=True, isaret=True)
            else:
                self.filo.git(rov_id, hedef[0], hedef[1], hedef[2], ai=True)
            rov = self.filo.find_rov_by_id(rov_id)
            if rov is not None:
                self._secim_markerlarini_guncelle(rov)
        except Exception as exc:
            self._last_command_error = exc
        self._gecici_marker_bitis = 0.0 if self.visible else monotonic() + 5.0
        return True

    def _gecici_marker_suresini_kontrol(self):
        if self.visible:
            return
        rov = self._aktif_marker_rov_al()
        if rov is not None:
            self._secim_markerlarini_guncelle(rov)
        elif self._rov_marker_id is not None or self._minimap_marker_id is not None:
            self._gecici_marker_bitis = 0.0
            self._secim_markerlarini_temizle()
            return
        if self._menu_root is not None or self._pending_action is not None:
            return
        if self._gecici_marker_bitis <= 0.0:
            return
        if monotonic() >= self._gecici_marker_bitis:
            self._gecici_marker_bitis = 0.0
            self._secim_markerlarini_temizle()

    def _aktif_marker_rov_al(self):
        rov_id = None
        if self._pending_action is not None:
            rov_id = self._pending_action.get("rov_id")
        elif self._menu_rov_id is not None:
            rov_id = self._menu_rov_id
        elif self._rov_marker_id is not None:
            rov_id = self._rov_marker_id
        if rov_id is None:
            return None
        try:
            return self.filo.find_rov_by_id(int(rov_id))
        except Exception:
            return None

    def _tiklanan_hedef_noktasi_al(self):
        rov = None
        if self._pending_action is not None:
            try:
                rov = self.filo.find_rov_by_id(int(self._pending_action.get("rov_id", -1)))
            except Exception:
                rov = None
        world_point = self._mouse_ray_havuz_kesisimi_al()
        if world_point is None:
            world_point = self._mouse_ray_duzlem_kesisimi_al(rov)
        if world_point is None:
            return None
        try:
            hedef_xyz = self._vec3_bilesenleri_al(world_point)
            if hedef_xyz is None:
                return None
            hedef_x = hedef_xyz[0]
            hedef_y = hedef_xyz[2]
        except Exception:
            return None
        hedef_z = self._rov_mevcut_derinligi_al(rov)
        return (hedef_x, hedef_y, hedef_z)

    def _mouse_ray_havuz_kesisimi_al(self):
        ortam = getattr(self.filo, "ortam_ref", None)
        plane_y = 0.0
        if ortam is not None:
            try:
                plane_y = float(getattr(ortam, "WATER_SURFACE_Y_BASE", 0.0))
            except Exception:
                plane_y = 0.0
        nokta = self._mouse_ray_dunya_duzlemi_kesisimi_al(plane_y)
        if nokta is None:
            return None
        if ortam is not None:
            try:
                sinir = float(getattr(ortam, "havuz_genisligi", 0.0) or 0.0)
            except Exception:
                sinir = 0.0
            if sinir > 0.0:
                nokta.x = max(-sinir, min(sinir, float(getattr(nokta, "x", 0.0))))
                nokta.z = max(-sinir, min(sinir, float(getattr(nokta, "z", 0.0))))
        return nokta

    def _mouse_ray_duzlem_kesisimi_al(self, rov=None):
        plane_y = 0.0
        if rov is not None:
            try:
                plane_y = float(getattr(rov, "y", 0.0))
            except Exception:
                plane_y = 0.0
        return self._mouse_ray_dunya_duzlemi_kesisimi_al(plane_y)

    def _mouse_ray_dunya_duzlemi_kesisimi_al(self, plane_y: float):
        origin = getattr(camera, "world_position", None)
        direction = getattr(mouse, "world_direction", None)
        if origin is None:
            return None
        if direction is None:
            direction = getattr(mouse, "ray", None)
        if direction is None:
            direction = getattr(camera, "forward", None)
        if direction is None:
            return None
        try:
            origin_xyz = self._vec3_bilesenleri_al(origin)
            if origin_xyz is None:
                return None
            ray_origin = Vec3(origin_xyz[0], origin_xyz[1], origin_xyz[2])
            ray_dir = self._normal_vec3_al(direction)
        except Exception:
            return None
        if ray_dir is None:
            return None
        ray_y = float(getattr(ray_dir, "y", 0.0))
        if abs(ray_y) < 0.0001:
            return None
        t = (plane_y - ray_origin.y) / ray_y
        if t <= 0.0:
            return None
        return ray_origin + (ray_dir * t)

    def _rov_mevcut_derinligi_al(self, rov):
        if rov is None:
            return 0.0
        try:
            gps = self.filo.get(self._rov_id_al(rov), "gps")
            if gps is not None and len(gps) >= 3:
                return float(gps[2])
        except Exception:
            pass
        try:
            return -float(getattr(rov, "y", 0.0))
        except Exception:
            return 0.0

    def _depth_klavye_guncelle(self, enter_down: bool):
        if enter_down and not self._enter_down:
            menu_rov_id = self._aktif_menu_rov_id_al()
            if menu_rov_id is None:
                self._menu_kapat()
                return
            try:
                rov = self.filo.find_rov_by_id(menu_rov_id)
            except Exception:
                rov = None
            if rov is not None:
                self._depth_komutunu_calistir(rov)
            return
        key_map = {
            "0": "0", "1": "1", "2": "2", "3": "3", "4": "4",
            "5": "5", "6": "6", "7": "7", "8": "8", "9": "9",
            "num 0": "0", "num 1": "1", "num 2": "2", "num 3": "3", "num 4": "4",
            "num 5": "5", "num 6": "6", "num 7": "7", "num 8": "8", "num 9": "9",
            ".": ".", "period": ".", "-": "-",
        }
        for key, char in key_map.items():
            pressed = self._tus_basili(key)
            if pressed and not self._depth_key_down.get(key, False):
                if char == "-" and self._depth_text:
                    continue
                if char == "." and "." in self._depth_text:
                    continue
                self._depth_text += char
                self._depth_menu_label_guncelle()
            self._depth_key_down[key] = pressed
        backspace = self._tus_basili("backspace")
        if backspace and not self._depth_key_down.get("backspace", False):
            self._depth_text = self._depth_text[:-1]
            self._depth_menu_label_guncelle()
        self._depth_key_down["backspace"] = backspace

    def _depth_menu_label_guncelle(self):
        if not self._menu_items:
            return
        text = self._depth_text if self._depth_text else "_"
        if "shadow" in self._menu_items[0]:
            self._menu_items[0]["shadow"].text = f"DEPTH: {text}"
        self._menu_items[0]["text"].text = f"DEPTH: {text}"

    def _depth_komutunu_calistir(self, rov):
        try:
            depth = float(self._depth_text)
        except (TypeError, ValueError):
            return
        if depth > 0.0:
            depth = -abs(depth)
        rov_id = self._rov_id_al(rov)
        try:
            gps = self.filo.get(rov_id, "gps")
            if gps is not None and len(gps) >= 2:
                self.filo.git(rov_id, float(gps[0]), float(gps[1]), depth, ai=True)
            else:
                self.filo.bat_gps(rov_id, depth)
        except Exception as exc:
            self._last_command_error = exc
        self._menu_kapat()
        self._secim_markerlarini_guncelle(rov)
        self._gecici_marker_bitis = 0.0 if self.visible else monotonic() + 5.0

    def _gosterilecek_rovleri_al(self):
        rovs = []
        if self.rov_ids is None:
            kaynak = getattr(self.filo, "rovs", []) or []
            rovs = [r for r in kaynak if r is not None]
        else:
            for rov_id in self.rov_ids:
                try:
                    rov = self.filo.find_rov_by_id(rov_id)
                except Exception:
                    rov = None
                if rov is not None:
                    rovs.append(rov)
        rovs.sort(key=lambda r: self._rov_id_al(r, 0))
        return rovs

    def _aktif_rov_al(self):
        rovs = self._gosterilecek_rovleri_al()
        if not rovs:
            self.selected_index = 0
            return None
        self.selected_index %= len(rovs)
        return rovs[self.selected_index]

    def _aktif_rov_id_al(self):
        rov = self._aktif_rov_al()
        if rov is None:
            return None
        try:
            return self._rov_id_al(rov)
        except Exception:
            return None

    def _sonraki_rov(self):
        rovs = self._gosterilecek_rovleri_al()
        if not rovs:
            self.selected_index = 0
            return
        self.selected_index = (self.selected_index + 1) % len(rovs)

    def _rows_enabled(self, enabled: bool):
        for row in self.rows:
            for key in ("label", "value", "track", "row_accent", "baseline", "line"):
                row[key].enabled = enabled
            for grid in row["grid_lines"]:
                grid.enabled = enabled

    def _panel_guncelle(self, rov):
        rov_id = self._rov_id_al(rov, 0)
        gnc = getattr(rov, "gnc", None)
        self.title.text = f"ROV-{rov_id} APF POWER"

        for row in self.rows:
            values = self._kuyruk_degerleri(gnc, row["attr"])
            son_deger = values[-1] if values else 0.0
            row["value"].text = f"{son_deger:.2f}"
            row["line"].enabled = bool(values)
            if not values:
                continue
            row["line"].model.vertices = self._vertices(values, row["x0"], row["y0"], row["width"], row["height"])
            row["line"].model.generate()

    def _secim_markerlarini_guncelle(self, rov):
        self._rov_marker_guncelle(rov)
        self._minimap_marker_guncelle(rov)

    def _secim_markerlarini_temizle(self):
        self._rov_marker_temizle()
        self._minimap_marker_temizle()

    def _rov_marker_guncelle(self, rov):
        rov_id = self._rov_id_al(rov)
        if self._rov_marker is not None and self._rov_marker_id == rov_id:
            self._rov_marker.enabled = True
            self._rov_marker_transform_guncelle(rov)
            return

        self._rov_marker_temizle()
        self._rov_marker_id = rov_id
        self._rov_marker = Entity(
            add_to_scene_entities=True,
            model=self._secim_kutusu_mesh_olustur(rov),
            color=color.white,
            enabled=True,
            unlit=True,
            double_sided=True,
        )
        self._rov_marker.alpha = 0.4
        self._rov_marker_transform_guncelle(rov)

    def _rov_marker_transform_guncelle(self, rov):
        if self._rov_marker is None:
            return
        self._rov_marker.position = Vec3(float(getattr(rov, "x", 0.0)), float(getattr(rov, "y", 0.0)), float(getattr(rov, "z", 0.0)))
        try:
            self._rov_marker.rotation = Vec3(
                float(getattr(rov, "rotation_x", 0.0)),
                float(getattr(rov, "rotation_y", 0.0)) + 90.0,
                float(getattr(rov, "rotation_z", 0.0)),
            )
        except Exception:
            self._rov_marker.rotation = (0, 90, 0)

    def _secim_kutusu_mesh_olustur(self, rov):
        sx, sy, sz = self._rov_dunya_olcegi_al(rov)
        hx = max(2.875, sx * 318.75)
        hy = max(1.375, sy * 152.50)
        hz = max(2.000, sz * 222.50)
        corners = (
            Vec3(-hx, -hy, -hz),
            Vec3(hx, -hy, -hz),
            Vec3(hx, -hy, hz),
            Vec3(-hx, -hy, hz),
            Vec3(-hx, hy, -hz),
            Vec3(hx, hy, -hz),
            Vec3(hx, hy, hz),
            Vec3(-hx, hy, hz),
        )
        edges = (
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        )
        vertices = []
        for a, b in edges:
            vertices.append(corners[a])
            vertices.append(corners[b])
        return Mesh(vertices=vertices, mode="lines", thickness=2, static=True)

    def _rov_dunya_olcegi_al(self, rov):
        try:
            scale = getattr(rov, "scale", Vec3(0.01, 0.01, 0.01))
            sx = abs(float(getattr(scale, "x", 0.01)))
            sy = abs(float(getattr(scale, "y", 0.01)))
            sz = abs(float(getattr(scale, "z", 0.01)))
        except Exception:
            sx = sy = sz = 0.01
        return sx, sy, sz

    def _rov_marker_temizle(self):
        if self._rov_marker is not None:
            try:
                destroy(self._rov_marker)
            except Exception:
                self._rov_marker.enabled = False
        self._rov_marker = None
        self._rov_marker_id = None

    def _minimap_marker_guncelle(self, rov):
        minimap = self._minimap_al()
        if minimap is None:
            self._minimap_marker_temizle()
            return

        rov_id = self._rov_id_al(rov)
        if self._minimap_marker is None or self._minimap_marker_id != rov_id:
            self._minimap_marker_temizle()
            self._minimap_marker_id = rov_id
            self._minimap_marker = Entity(
                parent=minimap,
                model=self._minimap_secim_karesi_mesh_olustur(),
                color=color.white,
                z=-0.62,
                enabled=True,
                unlit=True,
            )
            self._minimap_marker.alpha = 1.0
        self._minimap_marker_transform_guncelle(rov, minimap)

    def _minimap_marker_transform_guncelle(self, rov, minimap=None):
        if self._minimap_marker is None:
            return
        if minimap is None:
            minimap = self._minimap_al()
        if minimap is None:
            return
        try:
            target = minimap.dunya_to_harita(float(getattr(rov, "x", 0.0)), float(getattr(rov, "z", 0.0)))
            tx = float(getattr(target, "x", 0.0))
            ty = float(getattr(target, "y", 0.0))
            self._minimap_marker.position = (tx, ty, -0.62)
            self._minimap_marker.enabled = True
        except Exception:
            self._minimap_marker.enabled = False

    def _minimap_secim_karesi_mesh_olustur(self):
        h = 0.012
        vertices = [
            Vec3(-h, -h, 0), Vec3(h, -h, 0),
            Vec3(h, -h, 0), Vec3(h, h, 0),
            Vec3(h, h, 0), Vec3(-h, h, 0),
            Vec3(-h, h, 0), Vec3(-h, -h, 0),
        ]
        return Mesh(vertices=vertices, mode="lines", thickness=3, static=True)

    def _minimap_marker_temizle(self):
        if self._minimap_marker is not None:
            try:
                destroy(self._minimap_marker)
            except Exception:
                self._minimap_marker.enabled = False
        self._minimap_marker = None
        self._minimap_marker_id = None

    def _minimap_al(self):
        ortam = getattr(self.filo, "ortam_ref", None)
        return getattr(ortam, "minimap", None) if ortam is not None else None

    def _kuyruk_degerleri(self, gnc, attr: str):
        if gnc is None:
            return []
        try:
            raw_values = list(getattr(gnc, attr, []))[-self.HISTORY_LIMIT:]
        except TypeError:
            return []
        values = []
        for value in raw_values:
            try:
                values.append(max(0.0, min(1.0, float(value))))
            except (TypeError, ValueError):
                values.append(0.0)
        return values

    def _vertices(self, values, x0: float, y0: float, width: float, height: float):
        if len(values) == 1:
            values = [values[0], values[0]]
        offset = max(0, self.HISTORY_LIMIT - len(values))
        denom = float(max(1, self.HISTORY_LIMIT - 1))
        vertices = []
        for i, value in enumerate(values):
            x = x0 + ((offset + i) / denom) * width
            y = y0 + max(0.0, min(1.0, float(value))) * height
            vertices.append(Vec3(x, y, -0.06))
        return vertices
