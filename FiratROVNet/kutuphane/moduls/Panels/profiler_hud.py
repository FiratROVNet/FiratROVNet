from ursina import Entity, Text, camera, color, held_keys, mouse  # type: ignore[import]

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
        self.current_parent = None
        self.parent_stack = []
        self._display_rows = []
        self._row_hitboxes = []
        self._mouse_down = False
        self._back_down = False
        self._esc_down = False
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
            y = first_y - i * row_step
            hitbox = Entity(parent=self.root, model="quad", position=(0, y + 0.006, -0.04), scale=(1.06, 0.043), color=color.rgba(255, 255, 255, 1), collider="box")
            hitbox.alpha = 0.01
            rank = Text(parent=self.root, text=f"{i + 1:02d}", position=(-0.505, y, -0.03), origin=(-0.5, 0), scale=0.50, color=color.gray)
            name = Text(parent=self.root, text="-", position=(-0.445, y, -0.03), origin=(-0.5, 0), scale=0.50, color=color.white)
            avg = Text(parent=self.root, text="0.0", position=(0.245, y, -0.03), origin=(0.5, 0), scale=0.50, color=color.lime)
            last = Text(parent=self.root, text="0.0", position=(0.375, y, -0.03), origin=(0.5, 0), scale=0.50, color=color.yellow)
            maxv = Text(parent=self.root, text="0.0", position=(0.515, y, -0.03), origin=(0.5, 0), scale=0.50, color=color.red)
            self._row_hitboxes.append(hitbox)
            self.rows.append((rank, name, avg, last, maxv, hitbox))

    def set_visible(self, visible: bool):
        self.visible = bool(visible)
        self.root.enabled = self.visible

    def toggle(self):
        self.set_visible(not self.visible)

    def update(self, fps=None):
        if not self.visible:
            return

        self._input_guncelle()
        rows = self._aktif_satirlari_al()
        self._display_rows = rows
        top_n = min(len(self.rows), int(getattr(PerformansAyarlari, "PROFILER_TOP_N", len(self.rows))))
        fps_text = f" | FPS {fps:.1f}" if fps is not None else ""
        baslik = self.current_parent if self.current_parent is not None else "ROOT"
        if len(str(baslik)) > 46:
            baslik = "..." + str(baslik)[-43:]
        self.title.text = f"PROFILER HUD [{baslik}]{fps_text}"
        self.hint.text = "CLICK IN | BKSP/ESC OUT"

        for i, widgets in enumerate(self.rows):
            rank, name, avg, last, maxv, hitbox = widgets
            enabled = i < top_n and i < len(rows)
            for w in widgets:
                w.enabled = enabled
            if not enabled:
                continue
            row = rows[i]
            block_name = str(row["name"])
            child_count = int(row.get("child_count", 0) or 0)
            if child_count > 0:
                block_name = f"> {block_name}"
            if len(block_name) > 42:
                block_name = block_name[:39] + "..."
            rank.text = f"{i + 1:02d}"
            name.text = block_name
            name.color = color.azure if child_count > 0 else color.white
            avg.text = f"{row['avg'] * 1000:6.1f}"
            last.text = f"{row['last'] * 1000:6.1f}"
            maxv.text = f"{row['max'] * 1000:6.1f}"

    def _aktif_satirlari_al(self):
        snapshot = Profiler.snapshot()
        child_counts = {}
        for row in snapshot:
            parent = row.get("parent")
            if parent is not None:
                child_counts[parent] = child_counts.get(parent, 0) + 1
        rows = [row for row in snapshot if row.get("parent") == self.current_parent]
        for row in rows:
            row["child_count"] = child_counts.get(row.get("name"), 0)
        return sorted(rows, key=lambda r: (r["avg"], r["window_total"]), reverse=True)

    def _input_guncelle(self):
        hovered = getattr(mouse, "hovered_entity", None)
        left_down = bool(getattr(mouse, "left", False))
        back_down = self._tus_basili("backspace")
        esc_down = self._tus_basili("escape")

        if (back_down and not self._back_down) or (esc_down and not self._esc_down):
            self._geri_cik()
        self._back_down = back_down
        self._esc_down = esc_down

        if left_down and not self._mouse_down:
            self._tiklama_isle(hovered)
        self._mouse_down = left_down

    def input(self, key):
        if not self.visible:
            return
        if key in ("backspace", "escape"):
            self._geri_cik()
        elif key == "left mouse down" and not self._mouse_down:
            self._tiklama_isle(getattr(mouse, "hovered_entity", None))

    @staticmethod
    def _tus_basili(key):
        try:
            return bool(held_keys.get(key, 0))
        except Exception:
            return False

    def _tiklama_isle(self, hovered):
        row_index = self._hovered_row_index(hovered)
        if row_index is None:
            return
        self._satira_gir(row_index)

    def _hovered_row_index(self, hovered):
        if hovered is not None:
            current = hovered
            while current is not None:
                for index, hitbox in enumerate(self._row_hitboxes):
                    if current is hitbox:
                        return index
                current = getattr(current, "parent", None)
        return self._mouse_konumundan_satir_al()

    def _mouse_konumundan_satir_al(self):
        try:
            local_x = float(getattr(mouse, "x", 0.0)) - float(getattr(self.root, "x", 0.0))
            local_y = float(getattr(mouse, "y", 0.0)) - float(getattr(self.root, "y", 0.0))
        except Exception:
            return None

        if not (-0.53 <= local_x <= 0.53):
            return None
        for index, hitbox in enumerate(self._row_hitboxes):
            if not getattr(hitbox, "enabled", False):
                continue
            try:
                hy = float(getattr(hitbox, "y", 0.0))
                half_h = abs(float(getattr(getattr(hitbox, "scale", None), "y", 0.043))) * 0.5
            except Exception:
                continue
            if (hy - half_h) <= local_y <= (hy + half_h):
                return index
        return None

    def _satira_gir(self, row_index):
        if row_index < 0 or row_index >= len(self._display_rows):
            return
        row = self._display_rows[row_index]
        if int(row.get("child_count", 0) or 0) <= 0:
            return
        self.parent_stack.append(self.current_parent)
        self.current_parent = row.get("name")

    def _geri_cik(self):
        if self.parent_stack:
            self.current_parent = self.parent_stack.pop()
        else:
            self.current_parent = None
