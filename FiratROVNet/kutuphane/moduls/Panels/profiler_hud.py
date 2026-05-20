from ursina import Entity, Text, camera, color  # type: ignore[import]

from FiratROVNet.config import PerformansAyarlari
from FiratROVNet.kutuphane.moduls.profiler import Profiler


class ProfilerHUD:
    """Live profiler overlay for spotting frame bottlenecks without terminal spam."""

    def __init__(self, position=(-0.34, 0.04), visible=False):
        self.visible = bool(visible)
        self.position = position
        self.last_update = 0.0
        self.root = Entity(parent=camera.ui, position=(position[0], position[1], -8), enabled=self.visible)
        self.rows = []
        self._build()

    def _build(self):
        self.panel = Entity(parent=self.root, model="quad", scale=(1.12, 0.78), color=color.black, z=0.10)
        self.panel.alpha = 0.64

        header = Entity(parent=self.root, model="quad", position=(0, 0.335, -0.01), scale=(1.08, 0.064), color=color.black)
        header.alpha = 0.80

        accent = Entity(parent=self.root, model="quad", position=(-0.515, 0.335, -0.03), scale=(0.010, 0.044), color=color.orange)
        accent.alpha = 1.0

        border_color = color.orange
        for x, y, sx, sy in (
            (0, 0.390, 1.12, 0.002),
            (0, -0.390, 1.12, 0.002),
            (-0.560, 0, 0.002, 0.78),
            (0.560, 0, 0.002, 0.78),
        ):
            border = Entity(parent=self.root, model="quad", position=(x, y, -0.02), scale=(sx, sy), color=border_color)
            border.alpha = 0.62

        self.title = Text(
            parent=self.root,
            text="PROFILER HUD",
            position=(-0.495, 0.329, -0.03),
            origin=(-0.5, 0),
            scale=0.92,
            color=color.white,
        )
        self.hint = Text(
            parent=self.root,
            text="H TOGGLE",
            position=(0.505, 0.342, -0.03),
            origin=(0.5, 0),
            scale=0.50,
            color=color.white,
        )

        self.header = Text(
            parent=self.root,
            text="RANK  BLOCK                                      AVG(ms)   LAST(ms)    MAX(ms)",
            position=(-0.505, 0.270, -0.03),
            origin=(-0.5, 0),
            scale=0.50,
            color=color.azure,
        )

        first_y = 0.210
        row_step = 0.050
        row_count = max(6, int(getattr(PerformansAyarlari, "PROFILER_TOP_N", 18)))
        for i in range(row_count):
            rank = Text(parent=self.root, text=f"{i + 1:02d}", position=(-0.505, first_y - i * row_step, -0.03), origin=(-0.5, 0), scale=0.50, color=color.gray)
            name = Text(parent=self.root, text="-", position=(-0.445, first_y - i * row_step, -0.03), origin=(-0.5, 0), scale=0.50, color=color.white)
            avg = Text(parent=self.root, text="0.0", position=(0.245, first_y - i * row_step, -0.03), origin=(0.5, 0), scale=0.50, color=color.lime)
            last = Text(parent=self.root, text="0.0", position=(0.375, first_y - i * row_step, -0.03), origin=(0.5, 0), scale=0.50, color=color.yellow)
            maxv = Text(parent=self.root, text="0.0", position=(0.515, first_y - i * row_step, -0.03), origin=(0.5, 0), scale=0.50, color=color.red)
            self.rows.append((rank, name, avg, last, maxv))

    def set_visible(self, visible: bool):
        self.visible = bool(visible)
        self.root.enabled = self.visible

    def toggle(self):
        self.set_visible(not self.visible)

    def update(self, fps=None):
        if not self.visible:
            return

        rows = sorted(Profiler.snapshot(), key=lambda r: (r["avg"], r["window_total"]), reverse=True)
        top_n = min(len(self.rows), int(getattr(PerformansAyarlari, "PROFILER_TOP_N", len(self.rows))))
        fps_text = f" | FPS {fps:.1f}" if fps is not None else ""
        self.title.text = f"PROFILER HUD{fps_text}"

        for i, widgets in enumerate(self.rows):
            rank, name, avg, last, maxv = widgets
            enabled = i < top_n and i < len(rows)
            for w in widgets:
                w.enabled = enabled
            if not enabled:
                continue
            row = rows[i]
            block_name = str(row["name"])
            if len(block_name) > 42:
                block_name = block_name[:39] + "..."
            rank.text = f"{i + 1:02d}"
            name.text = block_name
            avg.text = f"{row['avg'] * 1000:6.1f}"
            last.text = f"{row['last'] * 1000:6.1f}"
            maxv.text = f"{row['max'] * 1000:6.1f}"
