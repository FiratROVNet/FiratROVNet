from ursina import Entity, Mesh, Text, Vec3, camera, color  # type: ignore[import]


class SACEgitimHUD:
    """SAC egitim metriklerini canli gosteren kompakt panel."""

    LOSS_METRICS = {"actor_loss", "critic_loss"}
    METRIC_STYLES = {
        "reward": ("REWARD", color.lime),
        "episode_reward": ("EP REWARD", color.lime),
        "best_episode_reward": ("BEST REW", color.green),
        "actor_loss": ("ACTOR LOSS", color.azure),
        "critic_loss": ("CRITIC LOSS", color.orange),
        "alpha": ("ALPHA", color.magenta),
        "buffer": ("BUFFER", color.white),
        "pitch_roll_odulu": ("PITCH ROLL", color.yellow),
        "pitch_reward": ("PITCH REW", color.yellow),
        "roll_reward": ("ROLL REW", color.red),
        "pitch_loss": ("PITCH LOSS", color.yellow),
        "roll_loss": ("ROLL LOSS", color.red),
    }

    def __init__(self, filo_ref, position=(0.43, -0.21), visible=False):
        self.filo = filo_ref
        self.visible = bool(visible)
        self.position = position
        self.selected_index = 0
        self.rov_ids = []
        self.root = Entity(parent=camera.ui, position=(position[0], position[1], -8), enabled=self.visible)
        self.rows = []
        self._build()

    def _build(self):
        self.panel = Entity(parent=self.root, model="quad", scale=(0.38, 0.48), color=color.black, z=0.10)
        self.panel.alpha = 0.46

        header = Entity(parent=self.root, model="quad", position=(0, 0.205, -0.01), scale=(0.368, 0.050), color=color.black)
        header.alpha = 0.74

        accent = Entity(parent=self.root, model="quad", position=(-0.174, 0.205, -0.03), scale=(0.006, 0.033), color=color.azure)
        accent.alpha = 1.0

        for x, y, sx, sy in (
            (0, 0.240, 0.38, 0.002),
            (0, -0.240, 0.38, 0.002),
            (-0.190, 0, 0.002, 0.48),
            (0.190, 0, 0.002, 0.48),
        ):
            border = Entity(parent=self.root, model="quad", position=(x, y, -0.02), scale=(sx, sy), color=color.azure)
            border.alpha = 0.72

        self.title = Text(
            parent=self.root,
            text="ROV-0 SAC TRAIN",
            position=(-0.160, 0.204, -0.03),
            origin=(-0.5, 0),
            scale=0.72,
            color=color.white,
        )
        self.hint = Text(
            parent=self.root,
            text="E TOGGLE   2 NEXT",
            position=(0.158, 0.214, -0.03),
            origin=(0.5, 0),
            scale=0.36,
            color=color.white,
        )
        self.hint.alpha = 0.92

        graph_width = 0.285
        graph_height = 0.058
        row_step = 0.064
        first_y = 0.132
        for row_i in range(6):
            center_y = first_y - row_i * row_step
            label = Text(
                parent=self.root,
                text="METRIC",
                position=(-0.166, center_y + 0.029, -0.03),
                origin=(-0.5, 0),
                scale=0.38,
                color=color.white,
            )
            value = Text(
                parent=self.root,
                text="0.000",
                position=(0.148, center_y + 0.029, -0.03),
                origin=(0.5, 0),
                scale=0.38,
                color=color.white,
            )
            track = Entity(parent=self.root, model="quad", position=(0.005, center_y - 0.010, -0.01), scale=(graph_width, graph_height * 0.84), color=color.black)
            track.alpha = 0.66
            baseline = Entity(parent=self.root, model="quad", position=(0.005, center_y - 0.010, -0.03), scale=(graph_width, 0.0015), color=color.white)
            baseline.alpha = 0.38
            mesh = Mesh(vertices=[Vec3(0, 0, 0), Vec3(0.001, 0, 0)], mode="line", thickness=3, static=False)
            line = Entity(parent=self.root, model=mesh, color=color.white, z=-0.05, enabled=False)
            self.rows.append(
                {
                    "metrik": None,
                    "label": label,
                    "value": value,
                    "line": line,
                    "track": track,
                    "baseline": baseline,
                    "x0": 0.005 - graph_width / 2.0,
                    "y0": center_y - 0.010 - graph_height / 2.0,
                    "width": graph_width,
                    "height": graph_height,
                }
            )

    def set_visible(self, visible: bool):
        self.visible = bool(visible)
        self.root.enabled = self.visible

    def toggle(self):
        self.set_visible(not self.visible)

    def set_rov_ids(self, rov_ids):
        self.rov_ids = [int(i) for i in rov_ids if i is not None]
        if self.selected_index >= len(self.rov_ids):
            self.selected_index = 0

    def active_rov_id(self):
        self._rov_ids_guncelle()
        if not self.rov_ids:
            return None
        return self.rov_ids[self.selected_index % len(self.rov_ids)]

    def next_rov(self):
        self._rov_ids_guncelle()
        if self.rov_ids:
            self.selected_index = (self.selected_index + 1) % len(self.rov_ids)
        return self.active_rov_id()

    def set_active_rov_id(self, rov_id: int | None):
        if rov_id is None:
            return None
        self._rov_ids_guncelle()
        try:
            hedef_rov_id = int(rov_id)
        except Exception:
            return None
        if hedef_rov_id not in self.rov_ids:
            self.rov_ids.append(hedef_rov_id)
            self.rov_ids.sort()
        self.selected_index = self.rov_ids.index(hedef_rov_id)
        return hedef_rov_id

    def update(self):
        if not self.visible:
            return
        rov_id = self.active_rov_id()
        if rov_id is None:
            self.title.text = "ROV-- SAC TRAIN"
            self._rows_enabled(False)
            return

        self.title.text = f"ROV-{rov_id} SAC TRAIN"
        self._rows_enabled(True)
        sac = getattr(self.filo, "sac", None)
        metrikler = sac.canli_egitim_metrikleri_al() if sac is not None else ()
        for index, row in enumerate(self.rows):
            if index >= len(metrikler):
                self._row_visible(row, False)
                continue
            metrik = metrikler[index]
            label_text, line_color = self.METRIC_STYLES.get(metrik, (str(metrik).upper(), color.white))
            self._row_visible(row, True)
            row["metrik"] = metrik
            row["label"].text = label_text
            row["label"].color = line_color
            row["value"].color = line_color
            row["line"].color = line_color
            row["warmup_steps"] = int(getattr(sac, "warmup_steps", 0) or 0) if sac is not None else 0
            values = sac.metrik_gecmisi(rov_id, metrik) if sac is not None else []
            self._row_guncelle(row, values)

    def _row_guncelle(self, row, values):
        if not values:
            row["value"].text = "0.000"
            row["line"].enabled = False
            return

        son_deger = float(values[-1])
        metrik = row.get("metrik")
        if metrik == "buffer":
            warmup_steps = int(row.get("warmup_steps", 0) or 0)
            row["value"].text = f"{int(son_deger)}/{warmup_steps}" if warmup_steps > 0 else f"{int(son_deger)}"
        else:
            row["value"].text = f"{son_deger:.3f}"
        cizilecek = list(values) if metrik in self.LOSS_METRICS else list(values[-120:])
        if len(cizilecek) == 1:
            cizilecek = [cizilecek[0], cizilecek[0]]
        elif len(cizilecek) < 2:
            row["line"].enabled = False
            return

        if metrik == "buffer" and int(row.get("warmup_steps", 0) or 0) > 0:
            max_abs = float(row["warmup_steps"])
        else:
            max_abs = max(max(abs(float(v)) for v in cizilecek), 1e-6)
        vertices = []
        adet = len(cizilecek)
        for i, value in enumerate(cizilecek):
            x = row["x0"] + (i / max(1, adet - 1)) * row["width"]
            normalized = max(-1.0, min(1.0, float(value) / max_abs))
            y = row["y0"] + (normalized + 1.0) * 0.5 * row["height"]
            vertices.append(Vec3(x, y, -0.05))
        row["line"].model.vertices = vertices
        row["line"].model.generate()
        row["line"].enabled = True

    def _rows_enabled(self, enabled: bool):
        for row in self.rows:
            self._row_visible(row, enabled)

    def _row_visible(self, row, enabled: bool):
        row["label"].enabled = enabled
        row["value"].enabled = enabled
        row["track"].enabled = enabled
        row["baseline"].enabled = enabled
        if not enabled:
            row["line"].enabled = False

    def _rov_ids_guncelle(self):
        try:
            sac = getattr(self.filo, "sac", None)
            ids = list(getattr(sac, "canli_egitim_rov_ids", []) or [])
            if not ids:
                ids = [int(getattr(rov, "id")) for rov in self.filo.rovs if rov and not getattr(rov, "is_destroyed", False)]
        except Exception:
            ids = []
        self.set_rov_ids(ids)
