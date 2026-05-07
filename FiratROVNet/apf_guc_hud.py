from time import monotonic

from ursina import Entity, Mesh, Text, Vec3, camera, color, destroy, held_keys, mouse  # type: ignore[import]


class APFGucHUD:
    """Tek ROV icin APF guc gecmisi paneli."""

    HISTORY_LIMIT = 150

    CHANNELS = (
        ("hedef_guc", "HEDEF_GUCU", color.lime),
        ("engel_guc", "ENGEL_GUCU", color.red),
        ("rov_guc", "ROV_GUCU", color.orange),
    )

    def __init__(self, filo_ref, position=(0.42, 0.30), visible=False):
        self.filo = filo_ref
        self.visible = bool(visible)
        self.position = position
        self.rov_ids = None
        self.selected_index = 0
        self._toggle_down = False
        self._next_down = False
        self._mouse_down = False
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
            self._rov_marker_temizle()
            self._minimap_marker_temizle()
        else:
            self._gecici_marker_bitis = 0.0

    def toggle(self):
        self.set_visible(not self.visible)

    def update(self, process_input: bool = True, draw: bool = True):
        if process_input:
            self._klavye_kontrol()
            self._fare_secim_kontrol()
            self._gecici_marker_suresini_kontrol()
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
        try:
            toggle_down = bool(held_keys.get("1", 0)) or bool(held_keys.get("num 1", 0))
            next_down = bool(held_keys.get("2", 0)) or bool(held_keys.get("num 2", 0))
        except Exception:
            toggle_down = False
            next_down = False

        if toggle_down and not self._toggle_down:
            self.toggle()
        self._toggle_down = toggle_down

        if self.visible and next_down and not self._next_down:
            self._sonraki_rov()
        self._next_down = next_down

    def _fare_secim_kontrol(self):
        try:
            down = bool(getattr(mouse, "left", False))
        except Exception:
            down = False

        if down and not self._mouse_down:
            rov = self._tiklanan_rov_al()
            if rov is not None:
                self._rov_sec(rov)
                self._secim_markerlarini_guncelle(rov)
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
            ray_origin = Vec3(float(origin.x), float(origin.y), float(origin.z))
            ray_dir = Vec3(float(direction.x), float(direction.y), float(direction.z)).normalized()
        except Exception:
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
                rov_pos = Vec3(float(pos.x), float(pos.y), float(pos.z))
            except Exception:
                continue
            ileri_mesafe = (rov_pos - ray_origin).dot(ray_dir)
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
            current = getattr(current, "parent", None)
        return False

    def _rov_sec(self, rov) -> bool:
        rov_id = int(getattr(rov, "id", -1))
        rovs = self._gosterilecek_rovleri_al()
        for i, aday in enumerate(rovs):
            if int(getattr(aday, "id", -2)) == rov_id:
                self.selected_index = i
                return True
        return False

    def _gecici_marker_suresini_kontrol(self):
        if self.visible or self._gecici_marker_bitis <= 0.0:
            return
        if self._rov_marker_id is not None:
            try:
                rov = self.filo.find_rov_by_id(int(self._rov_marker_id))
            except Exception:
                rov = None
            if rov is not None:
                self._secim_markerlarini_guncelle(rov)
            else:
                self._gecici_marker_bitis = 0.0
                self._secim_markerlarini_temizle()
                return
        if monotonic() >= self._gecici_marker_bitis:
            self._gecici_marker_bitis = 0.0
            self._secim_markerlarini_temizle()

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
        rovs.sort(key=lambda r: int(getattr(r, "id", 0)))
        return rovs

    def _aktif_rov_al(self):
        rovs = self._gosterilecek_rovleri_al()
        if not rovs:
            self.selected_index = 0
            return None
        self.selected_index %= len(rovs)
        return rovs[self.selected_index]

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
        rov_id = int(getattr(rov, "id", 0))
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
        rov_id = int(getattr(rov, "id", -1))
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
        return Mesh(vertices=vertices, mode="lines", thickness=2.5, static=True)

    def _rov_dunya_olcegi_al(self, rov):
        try:
            scale = getattr(rov, "scale", Vec3(0.01, 0.01, 0.01))
            sx = abs(float(scale.x))
            sy = abs(float(scale.y))
            sz = abs(float(scale.z))
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

        rov_id = int(getattr(rov, "id", -1))
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
            self._minimap_marker.position = (target.x, target.y, -0.62)
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
