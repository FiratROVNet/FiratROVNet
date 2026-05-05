from ursina import Entity, Text, camera, color  # type: ignore[import]


class MotorHUD:
    """Compact HUD for the selected ROV's motor powers."""

    MOTOR_NAMES = (
        "M0 FL", "M1 FR", "M2 RL", "M3 RR",
        "M4 VF", "M5 VF", "M6 VR", "M7 VR",
    )

    def __init__(self, filo_ref, position=(-0.69, -0.27), visible=True):
        self.filo = filo_ref
        self.visible = bool(visible)
        self.rov_id = None
        self.position = position

        self.root = Entity(parent=camera.ui, position=(position[0], position[1], -8), enabled=self.visible)
        self.rows = []

        self._build()

    def _build(self):
        self.panel = Entity(
            parent=self.root,
            model="quad",
            scale=(0.35, 0.43),
            color=color.black,
            z=0.10,
        )
        self.panel.alpha = 0.36
        border_color = color.gray
        borders = [
            Entity(parent=self.root, model="quad", position=(0, 0.215, -0.02), scale=(0.35, 0.002), color=border_color),
            Entity(parent=self.root, model="quad", position=(0, -0.215, -0.02), scale=(0.35, 0.002), color=border_color),
            Entity(parent=self.root, model="quad", position=(-0.175, 0, -0.02), scale=(0.002, 0.43), color=border_color),
            Entity(parent=self.root, model="quad", position=(0.175, 0, -0.02), scale=(0.002, 0.43), color=border_color),
        ]
        for border in borders:
            border.alpha = 0.48

        self.title = Text(
            parent=self.root,
            text="ROV-0 MOTOR POWER",
            position=(-0.157, 0.178, -0.03),
            origin=(-0.5, 0),
            scale=0.68,
            color=color.white,
        )
        Text(
            parent=self.root,
            text="HORIZONTAL",
            position=(-0.157, 0.126, -0.03),
            origin=(-0.5, 0),
            scale=0.46,
            color=color.azure,
        )
        Text(
            parent=self.root,
            text="VERTICAL",
            position=(-0.157, -0.055, -0.03),
            origin=(-0.5, 0),
            scale=0.46,
            color=color.lime,
        )

        row_y = [0.093, 0.052, 0.011, -0.030, -0.088, -0.129, -0.170, -0.211]
        for i, y in enumerate(row_y):
            label_color = color.azure if i < 4 else color.lime
            Text(
                parent=self.root,
                text=self.MOTOR_NAMES[i],
                position=(-0.157, y - 0.009, -0.03),
                origin=(-0.5, 0),
                scale=0.44,
                color=label_color,
            )
            track = Entity(
                parent=self.root,
                model="quad",
                position=(0.008, y, -0.02),
                scale=(0.142, 0.010),
                color=color.black,
            )
            track.alpha = 0.72
            center = Entity(
                parent=self.root,
                model="quad",
                position=(0.008, y, -0.04),
                scale=(0.002, 0.017),
                color=color.white,
            )
            center.alpha = 0.58

            pos_fill = Entity(
                parent=self.root,
                model="quad",
                position=(0.008, y, -0.05),
                scale=(0.001, 0.0075),
                color=color.lime,
            )
            pos_fill.alpha = 0.92
            pos_fill.origin = (-0.5, 0)

            neg_fill = Entity(
                parent=self.root,
                model="quad",
                position=(0.008, y, -0.05),
                scale=(0.001, 0.0075),
                color=color.red,
            )
            neg_fill.alpha = 0.86
            neg_fill.origin = (0.5, 0)

            value = Text(
                parent=self.root,
                text="+0.00",
                position=(0.095, y - 0.010, -0.03),
                origin=(-0.5, 0),
                scale=0.48,
                color=color.white,
            )
            self.rows.append((pos_fill, neg_fill, value))

    def set_visible(self, visible: bool):
        self.visible = bool(visible)
        self.root.enabled = self.visible

    def toggle(self):
        self.set_visible(not self.visible)

    def update(self, rov_id: int):
        if not self.visible:
            return

        if self.rov_id != rov_id:
            self.rov_id = rov_id
            self.title.text = f"ROV-{rov_id} MOTOR POWER"

        rov = None
        try:
            rov = self.filo.find_rov_by_id(rov_id)
        except Exception:
            rov = None

        motorlar = getattr(rov, "motorlar", []) if rov else []
        max_bar_width = 0.071

        for i, (pos_fill, neg_fill, value) in enumerate(self.rows):
            guc = 0.0
            if i < len(motorlar):
                try:
                    guc = float(getattr(motorlar[i], "guc", 0.0))
                except (TypeError, ValueError):
                    guc = 0.0
            guc = max(-1.0, min(1.0, guc))
            width = max(0.001, abs(guc) * max_bar_width)

            if guc > 0.01:
                pos_fill.scale_x = width
                neg_fill.scale_x = 0.001
                value.color = color.lime
            elif guc < -0.01:
                pos_fill.scale_x = 0.001
                neg_fill.scale_x = width
                value.color = color.red
            else:
                pos_fill.scale_x = 0.001
                neg_fill.scale_x = 0.001
                value.color = color.gray

            value.text = f"{guc:+.2f}"
