import os

from ursina import *  # type: ignore[import]


class BARUI:
    def __init__(self, app=None, root_position=(-0.58, 0.32), panel_scale=(0.46, 0.36)):
        self._owns_app = False
        self.app = app
        self._headless = self._headless_modda_mi()

        # If no app is provided, attach to an existing Ursina app when possible.
        if self.app is None and not self._headless:
            try:
                from ursina import application  # type: ignore[import]
                if getattr(application, 'base', None) is None:
                    self.app = Ursina()
                    self._owns_app = True
            except Exception:
                self._headless = True

        self.root_position = root_position
        self.panel_scale = panel_scale

        self._ui_root = None
        self.visible = False
        self.drag_state = None
        self.move_state = None

        self.sliders = {}
        self.labels = {}
        self.values = {}
        self.default_precision = 3

    @staticmethod
    def _headless_modda_mi() -> bool:
        for name in ("CI", "FIRAT_ROVNET_HEADLESS", "URSINA_HEADLESS", "QT_QPA_PLATFORM"):
            value = os.environ.get(name, "").lower()
            if value in {"1", "true", "yes", "on", "offscreen"}:
                return True
        return False

    def _get_precision(self, name: str):
        bar = self.sliders.get(name, {})
        p = bar.get('precision', self.default_precision)
        try:
            return max(0, int(p))
        except Exception:
            return self.default_precision

    def _format_value(self, name: str, value: float):
        p = self._get_precision(name)
        return f"{float(value):.{p}f}"

    def _create_bold_text(self, parent, text, position, scale, color_value, origin=(-0.5, 0), z=-0.05):
        """Simulates thicker text by drawing two overlapping Text entities."""
        main = Text(
            parent=parent,
            text=text,
            origin=origin,
            position=position,
            scale=scale,
            color=color_value,
            z=z,
        )
        bold = Text(
            parent=parent,
            text=text,
            origin=origin,
            position=(position[0] + 0.0009, position[1], position[2] if len(position) > 2 else z),
            scale=scale,
            color=color_value,
            z=z,
        )
        return {'main': main, 'bold': bold}

    def _set_value_label(self, name: str, value: float):
        label = self.labels.get(name)
        if not label:
            return
        text = f"{name}: {self._format_value(name, value)}"
        label['main'].text = text
        label['bold'].text = text

    def _get_bar_bounds(self, name: str):
        bar = self.sliders[name]
        if self._headless:
            return 0.0, 1.0
        center_x = float(bar['bar_root'].x) + float(bar['track_center_x'])
        half_width = float(bar['half_width'])
        return center_x - half_width, center_x + half_width

    @staticmethod
    def create_default_pid_bars(bar_ui: 'BARUI', callback=None):
        """Kp/Ki/Kd icin hazir bar seti olusturur."""
        bar_ui.create_bar('Kp', -1.0, 1.0, 1.0, (0.0, 0.07), callback=callback)
        bar_ui.create_bar('Ki', -1.0, 1.0, 1.0, (0.0, -0.015), callback=callback)
        bar_ui.create_bar('Kd', -1.0, 1.0, 1.0, (0.0, -0.10), callback=callback)

    def _ensure_root(self):
        if self._headless:
            return
        if self._ui_root is not None:
            return

        self._ui_root = Entity(parent=camera.ui, enabled=False, position=self.root_position)

    def create_bar(
        self,
        name: str,
        min_value: float,
        max_value: float,
        default: float,
        position: tuple,
        precision: int | None = None,
        callback=None,
    ):
        self._ensure_root()

        if max_value <= min_value:
            raise ValueError('max_value must be greater than min_value')

        x_pos, y_pos = float(position[0]), float(position[1])
        start_value = max(min_value, min(max_value, float(default)))
        p = self.default_precision if precision is None else max(0, int(precision))

        if self._headless:
            self.values[name] = round(start_value, p)
            self.sliders[name] = {
                'min_value': float(min_value),
                'max_value': float(max_value),
                'precision': p,
                'callback': callback,
            }
            return None

        bar_root = Entity(parent=self._ui_root, position=(x_pos, y_pos, 0))

        label = self._create_bold_text(
            parent=bar_root,
            text=f"{name}: {start_value:.{p}f}",
            position=(-0.21, 0.026, -0.05),
            scale=1.045,
            color_value=color.black,
            origin=(-0.5, 0),
        )
        self.labels[name] = label

        # Label bolgesinden bar'i tasimak icin gorunmez tasima alani
        move_handle = Button(  # type: ignore[call-arg]
            parent=bar_root,
            model='quad',  # type: ignore[arg-type]
            color=color.rgba(0, 0, 0, 0),  # type: ignore[arg-type]
            # Tasima sadece yazi golgesi/etiket bolgesinden baslasin.
            position=(-0.225, 0.026, -0.06),
            scale=(0.13, 0.04),
        )
        move_handle.highlight_color = color.rgba(0, 0, 0, 0)
        move_handle.pressed_color = color.rgba(0, 0, 0, 0)

        rail_bg = Entity(
            parent=bar_root,
            model='quad',
            color=color.rgba(255, 255, 255, 200),
            position=(0.02, 0, -0.008),
            scale=(0.34, 0.01),
        )
        rail_bg.ignore = True

        rail = Entity(
            parent=bar_root,
            model='quad',
            color=color.rgba(0, 0, 0, 230),
            position=(0.02, 0, -0.009),
            scale=(0.34, 0.0035),
        )
        rail.ignore = True

        self._create_bold_text(
            parent=bar_root,
            text=f"{min_value:g}",
            position=(-0.165, -0.018, -0.05),
            scale=0.77,
            color_value=color.rgba(30, 30, 30, 220),
            origin=(0.5, 0),
        )
        self._create_bold_text(
            parent=bar_root,
            text=f"{max_value:g}",
            position=(0.205, -0.018, -0.05),
            scale=0.77,
            color_value=color.rgba(30, 30, 30, 220),
            origin=(-0.5, 0),
        )

        track = Button(  # type: ignore[call-arg]
            parent=bar_root,
            model='quad',  # type: ignore[arg-type]
            color=color.rgba(0, 0, 0, 0),  # type: ignore[arg-type]
            position=(0.02, 0, -0.02),
            scale=(0.34, 0.05),
        )
        track.highlight_color = color.rgba(0, 0, 0, 0)
        track.pressed_color = color.rgba(0, 0, 0, 0)

        knob = Button(  # type: ignore[call-arg]
            parent=bar_root,
            model='quad',  # type: ignore[arg-type]
            color=color.red,  # type: ignore[arg-type]
            position=(0.02, 0, -0.03),
            scale=(0.015, 0.02),
            collider='box',
        )
        knob.highlight_color = color.orange
        knob.pressed_color = color.orange

        # Gorsel olarak ince kalan knob icin daha genis, gorunmez yakalama alani.
        drag_handle = Button(  # type: ignore[call-arg]
            parent=bar_root,
            model='quad',  # type: ignore[arg-type]
            color=color.rgba(0, 0, 0, 0),  # type: ignore[arg-type]
            position=(0.02, 0, -0.031),
            scale=(0.06, 0.05),
            collider='box',
        )
        drag_handle.highlight_color = color.rgba(0, 0, 0, 0)
        drag_handle.pressed_color = color.rgba(0, 0, 0, 0)

        track_center_x = 0.02
        half_width = 0.34 / 2
        min_x = float(bar_root.x) + track_center_x - half_width
        max_x = float(bar_root.x) + track_center_x + half_width

        self.values[name] = start_value
        knob.x = self._value_to_x(start_value, min_x, max_x, min_value, max_value) - float(bar_root.x)
        drag_handle.x = knob.x

        def _track_click(bar_name=name):
            if self._ui_root is None:
                return
            bar = self.sliders.get(bar_name)
            if not bar:
                return
            local_x = float(getattr(mouse, 'x', 0.0) - self._ui_root.x)
            clamped_x, value = self._update_value_from_x(bar_name, local_x)
            bar['knob'].x = clamped_x - float(bar['bar_root'].x)
            bar['drag_handle'].x = bar['knob'].x
            cb = bar.get('callback')
            if callable(cb):
                cb(value)

        def _knob_input(key, bar_name=name, knob_ref=knob):
            if key == 'left mouse down' and knob_ref.hovered:
                self.drag_state = bar_name
            elif key == 'left mouse up' and self.drag_state == bar_name:
                self.drag_state = None

        def _drag_handle_input(key, bar_name=name, handle_ref=drag_handle):
            if key == 'left mouse down' and handle_ref.hovered:
                self.drag_state = bar_name
            elif key == 'left mouse up' and self.drag_state == bar_name:
                self.drag_state = None

        def _track_input(key, bar_name=name, track_ref=track):
            if self._ui_root is None:
                return
            if key == 'left mouse down' and track_ref.hovered:
                self.drag_state = bar_name
                bar = self.sliders.get(bar_name)
                if not bar:
                    return
                local_x = float(getattr(mouse, 'x', 0.0) - self._ui_root.x)
                clamped_x, value = self._update_value_from_x(bar_name, local_x)
                bar['knob'].x = clamped_x - float(bar['bar_root'].x)
                bar['drag_handle'].x = bar['knob'].x
                cb = bar.get('callback')
                if callable(cb):
                    cb(value)
            elif key == 'left mouse up' and self.drag_state == bar_name:
                self.drag_state = None

        def _move_handle_input(key, bar_name=name):
            if self._ui_root is None:
                return
            bar = self.sliders.get(bar_name)
            if not bar:
                return
            # Knob/track etkileşimi varken tasimayi tetikleme.
            if bar['knob'].hovered or bar['drag_handle'].hovered or bar['track'].hovered:
                return
            if key == 'left mouse down' and move_handle.hovered:
                current_mouse_x = float(getattr(mouse, 'x', 0.0) - self._ui_root.x)
                current_mouse_y = float(getattr(mouse, 'y', 0.0) - self._ui_root.y)

                # Bar cizgisi araliginda tasima asla baslamasin.
                bar_min_x, bar_max_x = self._get_bar_bounds(bar_name)
                if bar_min_x <= current_mouse_x <= bar_max_x:
                    return

                self.move_state = {
                    'name': bar_name,
                    'offset_x': current_mouse_x - float(bar['bar_root'].x),
                    'offset_y': current_mouse_y - float(bar['bar_root'].y),
                }
            elif key == 'left mouse up' and self.move_state and self.move_state.get('name') == bar_name:
                self.move_state = None

        track.on_click = _track_click
        knob.input = _knob_input
        drag_handle.input = _drag_handle_input
        track.input = _track_input
        move_handle.input = _move_handle_input

        self.sliders[name] = {
            'bar_root': bar_root,
            'move_handle': move_handle,
            'rail_bg': rail_bg,
            'rail': rail,
            'track': track,
            'knob': knob,
            'drag_handle': drag_handle,
            'track_center_x': track_center_x,
            'half_width': half_width,
            'min_x': min_x,
            'max_x': max_x,
            'min_value': float(min_value),
            'max_value': float(max_value),
            'precision': p,
            'callback': callback,
        }

        return knob

    def _value_to_x(self, value: float, min_x: float, max_x: float, min_value: float, max_value: float):
        ratio = (float(value) - min_value) / (max_value - min_value)
        ratio = max(0.0, min(1.0, ratio))
        return min_x + ratio * (max_x - min_x)

    def _x_to_value(self, x: float, min_x: float, max_x: float, min_value: float, max_value: float):
        x_clamped = max(min_x, min(max_x, float(x)))
        ratio = (x_clamped - min_x) / (max_x - min_x)
        value = min_value + ratio * (max_value - min_value)
        return x_clamped, value

    def _update_value_from_x(self, name: str, x: float):
        bar = self.sliders[name]
        min_x, max_x = self._get_bar_bounds(name)
        bar['min_x'] = min_x
        bar['max_x'] = max_x
        clamped_x, value = self._x_to_value(
            x,
            min_x,
            max_x,
            bar['min_value'],
            bar['max_value'],
        )
        p = self._get_precision(name)
        rounded = round(value, p)
        self.values[name] = rounded
        self._set_value_label(name, rounded)
        return clamped_x, rounded

    def set_value(self, name: str, value: float):
        if name not in self.sliders:
            return
        bar = self.sliders[name]
        v = max(bar['min_value'], min(bar['max_value'], float(value)))
        p = self._get_precision(name)
        self.values[name] = round(v, p)
        if self._headless:
            return
        min_x, max_x = self._get_bar_bounds(name)
        bar['min_x'] = min_x
        bar['max_x'] = max_x
        bar['knob'].x = self._value_to_x(v, min_x, max_x, bar['min_value'], bar['max_value']) - float(bar['bar_root'].x)
        bar['drag_handle'].x = bar['knob'].x
        self._set_value_label(name, self.values[name])

    def get_value(self, name: str):
        return self.values.get(name)

    def toggle_ui(self, force=None):
        self._ensure_root()
        if force is None:
            self.visible = not self.visible
        else:
            self.visible = bool(force)
        if self._ui_root is not None:
            self._ui_root.enabled = self.visible

    def update(self):
        if not self.visible or self._ui_root is None:
            return

        if self.move_state is not None:
            bar_name = self.move_state.get('name')
            bar = self.sliders.get(bar_name)
            if not bar or not getattr(mouse, 'left', False):
                self.move_state = None
            else:
                local_x = float(getattr(mouse, 'x', 0.0) - self._ui_root.x)
                local_y = float(getattr(mouse, 'y', 0.0) - self._ui_root.y)
                bar['bar_root'].x = local_x - float(self.move_state['offset_x'])
                bar['bar_root'].y = local_y - float(self.move_state['offset_y'])

                # Bar tasininca knob'u mevcut degerle yeni konuma yeniden hizala
                self.set_value(bar_name, self.values.get(bar_name, 0.0))

        if self.drag_state is None:
            return

        bar = self.sliders.get(self.drag_state)
        if not bar or not getattr(mouse, 'left', False):
            self.drag_state = None
            return

        local_x = float(getattr(mouse, 'x', 0.0) - self._ui_root.x)
        clamped_x, value = self._update_value_from_x(self.drag_state, local_x)
        bar['knob'].x = clamped_x - float(bar['bar_root'].x)
        bar['drag_handle'].x = bar['knob'].x
        cb = bar.get('callback')
        if callable(cb):
            cb(value)

    def run(self):
        if self._owns_app and self.app is not None:
            self.app.run()
