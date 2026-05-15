# Panels

Tum oyun ici UI panelleri icin ortak klasor.

## Yeni panel ekleme

```python
panel = filo.panels.add(
    "telemetri",
    baslik="TELEMETRI",
    konum=(0.55, 0.25),
    olcek=(0.32, 0.18),
    saydamlik=0.45,
    kenarlik_saydamlik=0.65,
)
panel.add_text("gps", "GPS: 0, 0, 0", position=(-0.14, 0.03, -0.03))
panel.set_text("gps", "GPS: 12, -4, -20")
```

Turkce alan adlari desteklenir: `baslik`, `konum`, `olcek`,
`saydamlik`, `kenarlik`, `gorunur`.

## Grafik ekleme

```python
panel = filo.panels.add(
    "batarya_grafik",
    baslik="BATARYA",
    konum=(0.52, -0.05),
    olcek=(0.34, 0.22),
    saydamlik=0.42,
)

grafik = panel.add_chart(
    "batarya",
    position=(0.0, -0.02, -0.04),
    scale=(0.27, 0.10),
    x_range=(0, 180),
    y_range=(0, 100),
    max_points=180,
    show_grid=True,
    grid_steps=(6, 4),
    x_label="t",
    y_label="%",
)

grafik.append(86.4)
grafik.set_values([90, 88, 86, 87])
```

## Grafik renkleri ve coklu veri serisi

```python
from ursina import color

grafik = panel.add_chart(
    "enerji",
    position=(0.0, -0.02, -0.04),
    scale=(0.28, 0.12),
    y_range=(0, 100),
    show_axes=True,
    show_grid=True,
)

grafik.add_series("batarya", color_value=color.lime)
grafik.add_series("sicaklik", color_value=color.orange)

grafik.append(86.4, series="batarya", x=0)
grafik.append(42.0, series="sicaklik", x=0)

grafik.set_color(color.azure, series="batarya")
grafik.set_ranges(x_range=(0, 60), y_range=(0, 120))
```

## Goruntu paneli ekleme

```python
panel = filo.panels.add(
    "kamera",
    baslik="KAMERA",
    konum=(0.50, 0.25),
    olcek=(0.38, 0.28),
)

goruntu = panel.add_image(
    "rov_kamera",
    position=(0.0, -0.02, -0.04),
    scale=(0.32, 0.18),
)

goruntu.set_texture(texture)
```

## Ozel widget ekleme

```python
class BenimWidget:
    def __init__(self, parent, veri_kaynagi):
        self.parent = parent
        self.veri_kaynagi = veri_kaynagi

    def update(self):
        pass

panel.add_widget("ozel", BenimWidget, veri_kaynagi=filo)
```

## Mevcut paneli kaydetme

```python
motor_hud = filo.panels.register("motor_hud", MotorHUD(filo))
filo.panels.toggle("motor_hud")
filo.panels.set_visible("motor_hud", True)
```

Yeni panel kodlari bu klasorde tutulur. Repo icindeki panel importlari
`FiratROVNet.kutuphane.moduls.Panels` uzerinden yapilir.
