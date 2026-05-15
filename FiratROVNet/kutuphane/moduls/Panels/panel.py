from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Iterable

from ursina import Entity, Mesh, Text, Vec3, camera, color


@dataclass
class PanelStyle:
    position: tuple[float, float] = (0.0, 0.0)
    scale: tuple[float, float] = (0.35, 0.25)
    z: float = -8.0
    background_color: Any = color.black
    background_alpha: float = 0.42
    border_color: Any = color.azure
    border_alpha: float = 0.55
    border_width: float = 0.002
    title: str | None = None
    title_color: Any = color.white
    title_scale: float = 0.65
    title_position: tuple[float, float] | None = None
    visible: bool = True


class Panel:
    """Ursina UI icin ortak panel kabugu.

    Yeni bir panel eklemek icin:
        panel = panels.add("telemetri", title="TELEMETRI", position=(0.5, 0.2))
        panel.add_text("gps", "GPS: 0, 0, 0", position=(-0.15, 0.05))
    """

    def __init__(self, name: str, style: PanelStyle | None = None, parent=None):
        self.name = name
        self.style = style or PanelStyle()
        self.visible = bool(self.style.visible)
        self.root = Entity(
            parent=parent or camera.ui,
            position=(self.style.position[0], self.style.position[1], self.style.z),
            enabled=self.visible,
        )
        self.items: dict[str, Any] = {}
        self.widgets: dict[str, Any] = {}
        self._build_shell()

    def _build_shell(self):
        sx, sy = self.style.scale
        self.background = Entity(
            parent=self.root,
            model="quad",
            scale=(sx, sy),
            color=self.style.background_color,
            z=0.10,
        )
        self.background.alpha = self.style.background_alpha
        self.borders = (
            Entity(parent=self.root, model="quad", position=(0, sy / 2, -0.02), scale=(sx, self.style.border_width), color=self.style.border_color),
            Entity(parent=self.root, model="quad", position=(0, -sy / 2, -0.02), scale=(sx, self.style.border_width), color=self.style.border_color),
            Entity(parent=self.root, model="quad", position=(-sx / 2, 0, -0.02), scale=(self.style.border_width, sy), color=self.style.border_color),
            Entity(parent=self.root, model="quad", position=(sx / 2, 0, -0.02), scale=(self.style.border_width, sy), color=self.style.border_color),
        )
        for border in self.borders:
            border.alpha = self.style.border_alpha
        self.title = None
        if self.style.title:
            title_position = self.style.title_position or (-sx / 2 + 0.018, sy / 2 - 0.040)
            self.title = self.add_text(
                "title",
                self.style.title,
                position=(title_position[0], title_position[1], -0.03),
                scale=self.style.title_scale,
                color_value=self.style.title_color,
            )

    def add_text(self, name: str, text: str, position=(0, 0, -0.03), scale=0.5, color_value=None, origin=(-0.5, 0)):
        item = Text(
            parent=self.root,
            text=text,
            position=position,
            origin=origin,
            scale=scale,
            color=color.white if color_value is None else color_value,
        )
        self.items[name] = item
        return item

    def add_chart(self, name: str, **kwargs):
        chart = LineChart(parent=self.root, **kwargs)
        self.widgets[name] = chart
        return chart

    def add_image(self, name: str, **kwargs):
        image = ImageSlot(parent=self.root, **kwargs)
        self.widgets[name] = image
        return image

    def add_widget(self, name: str, widget_cls: type | Callable[..., Any], **kwargs):
        widget = widget_cls(parent=self.root, **kwargs)
        self.widgets[name] = widget
        return widget

    def widget(self, name: str, default=None):
        return self.widgets.get(name, default)

    def set_text(self, name: str, text: str):
        item = self.items.get(name)
        if item is not None:
            item.text = text

    def set_visible(self, visible: bool):
        self.visible = bool(visible)
        self.root.enabled = self.visible

    def toggle(self):
        self.set_visible(not self.visible)

    def set_position(self, position: tuple[float, float]):
        self.style.position = (float(position[0]), float(position[1]))
        self.root.position = (self.style.position[0], self.style.position[1], self.style.z)

    def update(self, *args, **kwargs):
        for widget in list(self.widgets.values()):
            updater = getattr(widget, "update", None)
            if callable(updater):
                updater(*args, **kwargs)


class LineChart:
    """Panel icinde hafif ve bagimsiz cizgi grafik widget'i.

    Tek seri:
        chart.append(12.5)

    Coklu seri:
        chart.add_series("batarya", color_value=color.lime)
        chart.append(86.4, series="batarya", x=12.0)
    """

    def __init__(
        self,
        parent,
        position=(0.0, 0.0, -0.04),
        scale=(0.25, 0.10),
        color_value=None,
        track_color=None,
        axis_color=None,
        grid_color=None,
        label_color=None,
        max_points: int = 120,
        x_range: tuple[float, float] | None = None,
        y_range: tuple[float, float] | None = None,
        thickness: int = 3,
        show_track: bool = True,
        show_baseline: bool = True,
        show_axes: bool = True,
        show_grid: bool = False,
        grid_steps: tuple[int, int] = (4, 3),
        x_label: str | None = None,
        y_label: str | None = None,
        title: str | None = None,
    ):
        self.parent = parent
        self.position = position
        self.scale = scale
        self.max_points = max(2, int(max_points))
        self.x_range = x_range
        self.y_range = y_range
        self.default_series = "default"
        self.series: dict[str, dict[str, Any]] = {}
        self.root = Entity(parent=parent, position=position)
        self.track = Entity(parent=self.root, model="quad", scale=scale, color=track_color or color.black, z=0.02, enabled=show_track)
        self.track.alpha = 0.48

        self.axis_color = axis_color or color.white
        self.grid_color = grid_color or color.gray
        self.label_color = label_color or color.white
        self.axis_entities = []
        self.grid_entities = []
        self.labels = {}
        self._build_axes(show_axes=show_axes, show_baseline=show_baseline, show_grid=show_grid, grid_steps=grid_steps)
        self._build_labels(title=title, x_label=x_label, y_label=y_label)
        self.add_series(self.default_series, color_value=color_value or color.lime, thickness=thickness)

    @property
    def values(self):
        return [point[1] for point in self.series[self.default_series]["points"]]

    @property
    def line(self):
        return self.series[self.default_series]["line"]

    def add_series(self, name: str, color_value=None, thickness: int = 3):
        if name in self.series:
            return self.series[name]["line"]
        mesh = Mesh(vertices=[Vec3(0, 0, 0), Vec3(0.001, 0, 0)], mode="line", thickness=thickness, static=False)
        line = Entity(parent=self.root, model=mesh, color=color_value or color.lime, z=-0.04, enabled=False)
        self.series[name] = {"points": [], "line": line}
        return line

    def append(self, value: float, *, series: str = "default", x: float | None = None):
        self.add_series(series)
        points = self.series[series]["points"]
        x_value = float(len(points) if x is None else x)
        points.append((x_value, float(value)))
        if len(points) > self.max_points:
            self.series[series]["points"] = points[-self.max_points:]
        self.redraw()

    def set_values(self, values: Iterable[float], *, series: str = "default", x_values: Iterable[float] | None = None):
        self.add_series(series)
        y_values = [float(value) for value in values]
        if x_values is None:
            points = [(float(index), value) for index, value in enumerate(y_values)]
        else:
            points = [(float(x), float(y)) for x, y in zip(x_values, y_values)]
        self.series[series]["points"] = points[-self.max_points:]
        self.redraw()

    def clear(self, series: str | None = None):
        names = [series] if series else list(self.series.keys())
        for name in names:
            if name in self.series:
                self.series[name]["points"].clear()
                self.series[name]["line"].enabled = False

    def set_color(self, color_value, *, series: str = "default"):
        self.add_series(series)
        self.series[series]["line"].color = color_value

    def set_ranges(self, *, x_range: tuple[float, float] | None = None, y_range: tuple[float, float] | None = None):
        if x_range is not None:
            self.x_range = x_range
        if y_range is not None:
            self.y_range = y_range
        self.redraw()

    def redraw(self):
        min_x, max_x = self._x_range()
        min_y, max_y = self._y_range()
        width, height = float(self.scale[0]), float(self.scale[1])
        x0, y0 = -width / 2.0, -height / 2.0
        x_span = max(max_x - min_x, 1e-9)
        y_span = max(max_y - min_y, 1e-9)

        for data in self.series.values():
            points = data["points"]
            line = data["line"]
            if len(points) < 2:
                line.enabled = False
                continue
            vertices = []
            for x_value, y_value in points:
                x_ratio = max(0.0, min(1.0, (float(x_value) - min_x) / x_span))
                y_ratio = max(0.0, min(1.0, (float(y_value) - min_y) / y_span))
                x = x0 + x_ratio * width
                y = y0 + y_ratio * height
                vertices.append(Vec3(x, y, -0.04))
            line.model.vertices = vertices
            line.model.generate()
            line.enabled = True

    def _build_axes(self, show_axes: bool, show_baseline: bool, show_grid: bool, grid_steps: tuple[int, int]):
        width, height = float(self.scale[0]), float(self.scale[1])
        if show_baseline:
            baseline = Entity(parent=self.root, model="quad", position=(0, 0, -0.02), scale=(width, 0.0015), color=self.axis_color)
            baseline.alpha = 0.35
            self.axis_entities.append(baseline)
        if show_axes:
            x_axis = Entity(parent=self.root, model="quad", position=(0, -height / 2, -0.025), scale=(width, 0.0015), color=self.axis_color)
            y_axis = Entity(parent=self.root, model="quad", position=(-width / 2, 0, -0.025), scale=(0.0015, height), color=self.axis_color)
            x_axis.alpha = 0.48
            y_axis.alpha = 0.48
            self.axis_entities.extend([x_axis, y_axis])
        if show_grid:
            x_steps, y_steps = max(1, int(grid_steps[0])), max(1, int(grid_steps[1]))
            for index in range(1, x_steps):
                x = -width / 2 + (index / x_steps) * width
                grid = Entity(parent=self.root, model="quad", position=(x, 0, -0.015), scale=(0.001, height), color=self.grid_color)
                grid.alpha = 0.24
                self.grid_entities.append(grid)
            for index in range(1, y_steps):
                y = -height / 2 + (index / y_steps) * height
                grid = Entity(parent=self.root, model="quad", position=(0, y, -0.015), scale=(width, 0.001), color=self.grid_color)
                grid.alpha = 0.24
                self.grid_entities.append(grid)

    def _build_labels(self, title: str | None, x_label: str | None, y_label: str | None):
        width, height = float(self.scale[0]), float(self.scale[1])
        if title:
            self.labels["title"] = Text(parent=self.root, text=title, position=(-width / 2, height / 2 + 0.012, -0.03), origin=(-0.5, 0), scale=0.34, color=self.label_color)
        if x_label:
            self.labels["x"] = Text(parent=self.root, text=x_label, position=(width / 2, -height / 2 - 0.023, -0.03), origin=(0.5, 0), scale=0.28, color=self.label_color)
        if y_label:
            self.labels["y"] = Text(parent=self.root, text=y_label, position=(-width / 2 - 0.010, height / 2 - 0.008, -0.03), origin=(0.5, 0), scale=0.28, color=self.label_color)

    def _all_points(self):
        points = []
        for data in self.series.values():
            points.extend(data["points"])
        return points

    def _x_range(self):
        if self.x_range is not None:
            return float(self.x_range[0]), float(self.x_range[1])
        points = self._all_points()
        if not points:
            return 0.0, 1.0
        min_x = min(point[0] for point in points)
        max_x = max(point[0] for point in points)
        if min_x == max_x:
            return min_x, min_x + 1.0
        return min_x, max_x

    def _y_range(self):
        if self.y_range is not None:
            return float(self.y_range[0]), float(self.y_range[1])
        points = self._all_points()
        if not points:
            return 0.0, 1.0
        min_y = min(point[1] for point in points)
        max_y = max(point[1] for point in points)
        if min_y == max_y:
            padding = max(abs(min_y) * 0.1, 1.0)
            return min_y - padding, max_y + padding
        padding = (max_y - min_y) * 0.08
        return min_y - padding, max_y + padding


class ImageSlot:
    """Panel icinde texture/goruntu gostermek icin alan."""

    def __init__(
        self,
        parent,
        texture=None,
        position=(0.0, 0.0, -0.04),
        scale=(0.20, 0.14),
        background_color=None,
        background_alpha: float = 0.35,
        fit: str = "stretch",
    ):
        self.fit = fit
        self.root = Entity(parent=parent, position=position)
        self.background = Entity(parent=self.root, model="quad", scale=scale, color=background_color or color.black, z=0.02)
        self.background.alpha = background_alpha
        self.image = Entity(parent=self.root, model="quad", texture=texture, scale=scale, z=-0.02)

    def set_texture(self, texture):
        self.image.texture = texture

    def set_visible(self, visible: bool):
        self.root.enabled = bool(visible)


class PanelManager:
    """Tum UI panellerini tek yerden ekleme, bulma, gosterme ve guncelleme."""

    def __init__(self):
        self.panels: dict[str, Any] = {}

    def add(
        self,
        name: str,
        panel_cls: type | None = None,
        *,
        builder: Callable[[Panel], Any] | None = None,
        instance: Any | None = None,
        **style_kwargs,
    ):
        style_kwargs = self._normalize_style_kwargs(style_kwargs)
        if name in self.panels:
            raise ValueError(f"Panel zaten var: {name}")
        if instance is not None:
            self.panels[name] = instance
            return instance
        if panel_cls is not None:
            panel = panel_cls(**style_kwargs)
        else:
            style = PanelStyle(**style_kwargs)
            panel = Panel(name=name, style=style)
            if callable(builder):
                builder(panel)
        self.panels[name] = panel
        return panel

    def _normalize_style_kwargs(self, kwargs: dict[str, Any]):
        aliases = {
            "konum": "position",
            "pozisyon": "position",
            "olcek": "scale",
            "boyut": "scale",
            "baslik": "title",
            "goster": "visible",
            "gorunur": "visible",
            "saydamlik": "background_alpha",
            "arka_plan_saydamlik": "background_alpha",
            "arkaplan_saydamlik": "background_alpha",
            "arka_plan": "background_color",
            "arkaplan": "background_color",
            "kenarlik": "border_color",
            "kenarlik_saydamlik": "border_alpha",
        }
        normalized = {}
        for key, value in kwargs.items():
            normalized[aliases.get(key, key)] = value
        return normalized

    def register(self, name: str, panel: Any):
        self.panels[name] = panel
        return panel

    def get(self, name: str, default=None):
        return self.panels.get(name, default)

    def remove(self, name: str):
        panel = self.panels.pop(name, None)
        root = getattr(panel, "root", None) or getattr(panel, "_ui_root", None)
        if root is not None:
            root.enabled = False
        return panel

    def set_visible(self, name: str, visible: bool):
        panel = self.get(name)
        if panel is None:
            return None
        if hasattr(panel, "set_visible"):
            panel.set_visible(bool(visible))
        elif hasattr(panel, "toggle_ui"):
            panel.toggle_ui(bool(visible))
        elif hasattr(panel, "root"):
            panel.root.enabled = bool(visible)
            panel.visible = bool(visible)
        return getattr(panel, "visible", visible)

    def toggle(self, name: str):
        panel = self.get(name)
        if panel is None:
            return None
        if hasattr(panel, "toggle"):
            panel.toggle()
        elif hasattr(panel, "toggle_ui"):
            panel.toggle_ui()
        else:
            self.set_visible(name, not bool(getattr(panel, "visible", True)))
        return getattr(panel, "visible", None)

    def update(self, *args, **kwargs):
        for panel in list(self.panels.values()):
            updater = getattr(panel, "update", None)
            if callable(updater):
                updater(*args, **kwargs)
