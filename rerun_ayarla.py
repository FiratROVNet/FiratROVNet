import os
import socket
import threading
from io import StringIO
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import quote, urlsplit, urlunsplit

import numpy as np
import rerun as rr

try:
    import qrcode
except ImportError:
    qrcode = None

_RERUN_ENGEL_RADIUS = 0.35
_RERUN_ADA_RENK = np.array([[101, 52, 17]], dtype=np.uint8)
_RERUN_ADA_CEVRE_RENK = np.array([[101, 52, 17]], dtype=np.uint8)
_RERUN_ADA_CEVRE_RADIUS = 1.44
_RERUN_ROV_HALF_SIZE = np.array([0.9, 0.45, 0.9], dtype=np.float32)
_RERUN_ENGEL_RENK_ACIK = np.array([205, 170, 125], dtype=np.float32)
_RERUN_ENGEL_RENK_KOYU = np.array([101, 52, 17], dtype=np.float32)


def _start_rr_alias_server(bind_host, bind_port, route_path, redirect_url):

    class _AliasHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            req_path = self.path.split('?', 1)[0]
            if req_path == route_path:
                self.send_response(302)
                self.send_header("Location", redirect_url)
                self.end_headers()
                return

            self.send_response(404)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write(b"Not Found")

        def log_message(self, format, *args):
            return

    alias_server = ThreadingHTTPServer((bind_host, bind_port), _AliasHandler)
    alias_thread = threading.Thread(target=alias_server.serve_forever, daemon=True)
    alias_thread.start()
    return alias_server


def _detect_lan_ip(fallback_ip="127.0.0.1"):
    lan_ip = fallback_ip
    ip_probe = None
    try:
        ip_probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ip_probe.connect(("8.8.8.8", 80))
        lan_ip = ip_probe.getsockname()[0]
    except OSError:
        pass
    finally:
        if ip_probe is not None:
            ip_probe.close()
    return lan_ip


def _uri_host_degistir(uri, yeni_host):
    """URI icindeki host bilgisini guvenli sekilde degistirir."""
    uri_str = str(uri or "")
    if not uri_str:
        return uri_str

    try:
        parsed = urlsplit(uri_str)
        if not parsed.netloc:
            # URI parse edilemiyorsa eski davranişa guvenli fallback.
            return uri_str.replace("127.0.0.1", yeni_host).replace("localhost", yeni_host)

        userinfo = ""
        if parsed.username:
            userinfo = parsed.username
            if parsed.password:
                userinfo = f"{userinfo}:{parsed.password}"
            userinfo = f"{userinfo}@"

        host_parcasi = str(yeni_host)
        if ":" in host_parcasi and not host_parcasi.startswith("["):
            host_parcasi = f"[{host_parcasi}]"

        yeni_netloc = f"{userinfo}{host_parcasi}"
        if parsed.port is not None:
            yeni_netloc = f"{yeni_netloc}:{parsed.port}"

        return urlunsplit((parsed.scheme, yeni_netloc, parsed.path, parsed.query, parsed.fragment))
    except Exception:
        return uri_str.replace("127.0.0.1", yeni_host).replace("localhost", yeni_host)


def QR(ip_adresi, link, show=False):
    """Terminale QR basar; show=True ise buyuk bir Python penceresinde de gosterir."""
    ip_adresi = str(ip_adresi or "-")
    link = str(link or "")

    if not link:
        print("[Rerun][QR] Link bos oldugu icin QR olusturulamadi.")
        return None

    if qrcode is None:
        print("[Rerun] QR icin qrcode paketi bulunamadi. URL'yi elle acabilirsiniz.")
        print(f"[Rerun][QR] IP: {ip_adresi}")
        print(f"[Rerun][QR] Link: {link}")
        return None

    qr = qrcode.QRCode(border=1)
    qr.add_data(link)
    qr.make(fit=True)

    qr_buf = StringIO()
    qr.print_ascii(out=qr_buf, invert=True)
    print(f"[Rerun][QR] IP: {ip_adresi}")
    print(f"[Rerun][QR] Link: {link}")
    print("[Rerun][QR] Terminal QR:")
    print(qr_buf.getvalue())

    qr_img = qr.make_image(fill_color="black", back_color="white")

    if show:
        try:
            import matplotlib.pyplot as plt

            qr_matrix = np.asarray(qr.get_matrix(), dtype=np.uint8)
            qr_np = np.where(qr_matrix == 1, 0, 255).astype(np.uint8)
            qr_np = np.kron(qr_np, np.ones((12, 12), dtype=np.uint8))
            plt.figure("Rerun QR", figsize=(8, 8))
            plt.imshow(qr_np, cmap="gray", interpolation="nearest")
            plt.title(f"Rerun QR - {ip_adresi}")
            plt.axis("off")
            plt.tight_layout()
            plt.show(block=False)
            plt.pause(0.001)
        except Exception as exc:
            print(f"[Rerun][QR] Pencere acilamadi: {exc}")

    return qr_img


def rerun_baslat(ip_adresi=None):
    # Gömülü viewer davranisini başlatmadan önce zorla.
    os.environ["RERUN_VIEWER_MOBILE_WARNING"] = "1"
    os.environ["RERUN_FORCE_DESKTOP"] = "1"

    rr.init("Otonom_Arac_Lidar_Simulasyonu")

    rr_grpc_port = int(os.getenv("RR_GRPC_PORT", "9876"))
    rr_web_port = int(os.getenv("RR_WEB_PORT", "9091"))
    rr_alias_port = int(os.getenv("RR_ALIAS_PORT", "9090"))
    rr_alias_path = os.getenv("RR_ALIAS_PATH", "/FiratROVNet")
    open_browser = os.getenv("RR_OPEN_BROWSER", "false").lower() in ("1", "true", "yes")

    lan_ip = ip_adresi or _detect_lan_ip()

    server_uri = str(rr.serve_grpc(grpc_port=rr_grpc_port))
    server_uri_lan = _uri_host_degistir(server_uri, lan_ip)
    rr.serve_web_viewer(
        web_port=rr_web_port, 
        connect_to=server_uri_lan, 
        open_browser=open_browser,
        # Eğer destekleniyorsa bu parametreleri ekle
        # disable_mobile_warning=True,
        # force_desktop=True
    )

    web_local_url = f"http://127.0.0.1:{rr_web_port}/?url={quote(server_uri, safe='')}"
    web_lan_url = f"http://{lan_ip}:{rr_web_port}/?url={quote(server_uri_lan, safe='')}"
    alias_local_url = f"http://127.0.0.1:{rr_alias_port}{rr_alias_path}"
    alias_lan_url = f"http://{lan_ip}:{rr_alias_port}{rr_alias_path}"

    alias_server = None
    if rr_alias_port == rr_web_port:
        print("[Rerun] RR_ALIAS_PORT ve RR_WEB_PORT ayni oldugu icin alias route acilamadi.")
        print("[Rerun] Farkli port ver: RR_WEB_PORT=9091 RR_ALIAS_PORT=9090")
    else:
        alias_server = _start_rr_alias_server("0.0.0.0", rr_alias_port, rr_alias_path, web_lan_url)
        print(f"[Rerun] Alias: {alias_lan_url}")

    print(f"[Rerun] Web: {web_lan_url}")

    return {
        "lan_ip": lan_ip,
        "server_uri": server_uri,
        "server_uri_lan": server_uri_lan,
        "web_local_url": web_local_url,
        "web_lan_url": web_lan_url,
        "alias_local_url": alias_local_url,
        "alias_lan_url": alias_lan_url,
        "alias_server": alias_server,
    }


def _ursina_to_rerun_xyz(x, y, z):
    return (float(x), float(z), float(y))


def _ursina_to_rerun_half_sizes(x, y, z):
    return (float(x), float(z), float(y))


def _engel_bulutu_to_points3d(engel_bulutu):
    if engel_bulutu is None:
        return np.empty((0, 3), dtype=np.float32)

    if isinstance(engel_bulutu, np.ndarray):
        arr = np.asarray(engel_bulutu, dtype=np.float32)
        if arr.ndim == 1:
            if arr.size < 3:
                return np.empty((0, 3), dtype=np.float32)
            arr = arr.reshape(1, -1)
        if arr.ndim != 2 or arr.shape[1] < 3:
            return np.empty((0, 3), dtype=np.float32)
        # Girdi dizi formati x,z,y kabul edilir ve Rerun eksenine x,z,y olarak aktarilir.
        return arr[:, [0, 1, 2]]

    points = []
    for nokta in engel_bulutu:
        if nokta is None:
            continue
        try:
            if len(nokta) >= 4 and isinstance(nokta[3], str):
                points.append(_ursina_to_rerun_xyz(nokta[0], nokta[2], nokta[1]))
            elif len(nokta) >= 3:
                points.append(_ursina_to_rerun_xyz(nokta[0], nokta[1], nokta[2]))
        except (TypeError, ValueError):
            continue

    if not points:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray(points, dtype=np.float32)


def _engel_renkleri_hesapla(yukseklikler, sea_floor_y, surface_y):
    if yukseklikler is None or len(yukseklikler) == 0:
        return np.empty((0, 3), dtype=np.uint8)

    yuk = np.asarray(yukseklikler, dtype=np.float32)
    dip = float(sea_floor_y)
    yuzey = float(surface_y)
    if yuzey <= dip:
        oran = np.ones_like(yuk, dtype=np.float32)
    else:
        oran = np.clip((yuk - dip) / (yuzey - dip), 0.0, 1.0)

    # Dibe yakin: acik kahve, yuzeye yakin: adalarla ayni koyu kahve.
    renkler = _RERUN_ENGEL_RENK_ACIK + (_RERUN_ENGEL_RENK_KOYU - _RERUN_ENGEL_RENK_ACIK) * oran.reshape(-1, 1)
    return np.clip(np.rint(renkler), 0, 255).astype(np.uint8)


def _rovlar_to_rerun_arrays(rovs):
    centers = []
    colors = []

    for rov in rovs or []:
        if rov is None or getattr(rov, "is_destroyed", False):
            continue
        try:
            centers.append(_ursina_to_rerun_xyz(rov.position.x, rov.position.y, rov.position.z))
        except (AttributeError, TypeError, ValueError):
            continue

        rov_color = getattr(rov, "color", None)
        try:
            r = int(max(0, min(255, round(float(getattr(rov_color, "r", 1.0)) * 255))))
            g = int(max(0, min(255, round(float(getattr(rov_color, "g", 1.0)) * 255))))
            b = int(max(0, min(255, round(float(getattr(rov_color, "b", 1.0)) * 255))))
        except (AttributeError, TypeError, ValueError):
            r, g, b = 255, 255, 255
        colors.append((r, g, b))

    if not centers:
        return (
            np.empty((0, 3), dtype=np.float32),
            np.empty((0, 3), dtype=np.uint8),
            np.empty((0, 3), dtype=np.float32),
        )

    centers_np = np.asarray(centers, dtype=np.float32)
    colors_np = np.asarray(colors, dtype=np.uint8)
    rov_half_size_rr = np.asarray(_ursina_to_rerun_half_sizes(*_RERUN_ROV_HALF_SIZE), dtype=np.float32)
    half_sizes_np = np.repeat(rov_half_size_rr.reshape(1, 3), centers_np.shape[0], axis=0)
    return centers_np, colors_np, half_sizes_np


def _rr_set_step(step):
    set_time_sequence_fn = getattr(rr, "set_time_sequence", None)
    if callable(set_time_sequence_fn):
        set_time_sequence_fn("step", int(step))
        return
    rr.set_time("step", sequence=int(step))


def rerun_sahne_logla(app, filo, step):
    engel_points = _engel_bulutu_to_points3d(getattr(app, "engel_bulutu", None))
    surface_y = float(getattr(app, "WATER_SURFACE_Y_BASE", 0.0))
    sea_floor_y = float(getattr(app, "SEA_FLOOR_Y", -50.0))
    _rr_set_step(step)

    if engel_points.shape[0] == 0:
        rr.log(
            "engeller",
            rr.Points3D(
                engel_points,
                colors=np.empty((0, 3), dtype=np.uint8),
                radii=np.empty((0,), dtype=np.float32),
            ),
        )
    else:
        # Rerun nokta duzeni (x, z, y) oldugu icin Ursina'nin derinlik ekseni Y her zaman index=2'dedir.
        engel_ursina_y = engel_points[:, 2]
        colors = _engel_renkleri_hesapla(engel_ursina_y, sea_floor_y=sea_floor_y, surface_y=surface_y)
        radii = np.full((engel_points.shape[0],), _RERUN_ENGEL_RADIUS, dtype=np.float32)
        rr.log("engeller", rr.Points3D(engel_points, colors=colors, radii=radii))

    ada_merkezleri = []
    ada_radii = []
    for ada in getattr(app, "island_positions", []) or []:
        if ada is None or len(ada) < 3:
            continue
        try:
            ada_x = float(ada[0])
            ada_z = float(ada[1])
            ada_radius = float(ada[2])
        except (TypeError, ValueError):
            continue
        ada_merkezleri.append(_ursina_to_rerun_xyz(ada_x, surface_y + 0.35, ada_z))
        ada_radii.append(max(1.4, ada_radius * 0.18))

    if ada_merkezleri:
        ada_merkezleri_np = np.asarray(ada_merkezleri, dtype=np.float32)
        ada_radii_np = np.asarray(ada_radii, dtype=np.float32)
        ada_colors = np.repeat(_RERUN_ADA_RENK, ada_merkezleri_np.shape[0], axis=0)
        rr.log("adalar/merkezler", rr.Points3D(ada_merkezleri_np, colors=ada_colors, radii=ada_radii_np))
    else:
        rr.log(
            "adalar/merkezler",
            rr.Points3D(
                np.empty((0, 3), dtype=np.float32),
                colors=np.empty((0, 3), dtype=np.uint8),
                radii=np.empty((0,), dtype=np.float32),
            ),
        )

    ada_cevre_noktalari = []
    for nokta in filo.ada_cevre(sessiz=True) or []:
        if nokta is None or len(nokta) < 2:
            continue
        try:
            nokta_x = float(nokta[0])
            nokta_z = float(nokta[1])
        except (TypeError, ValueError):
            continue
        ada_cevre_noktalari.append(_ursina_to_rerun_xyz(nokta_x, surface_y + 0.35, nokta_z))

    if ada_cevre_noktalari:
        ada_cevre_np = np.asarray(ada_cevre_noktalari, dtype=np.float32)
        ada_cevre_colors = np.repeat(_RERUN_ADA_CEVRE_RENK, ada_cevre_np.shape[0], axis=0)
        ada_cevre_radii = np.full((ada_cevre_np.shape[0],), _RERUN_ADA_CEVRE_RADIUS, dtype=np.float32)
        rr.log("adalar/cevre_noktalari", rr.Points3D(ada_cevre_np, colors=ada_cevre_colors, radii=ada_cevre_radii))
    else:
        rr.log(
            "adalar/cevre_noktalari",
            rr.Points3D(
                np.empty((0, 3), dtype=np.float32),
                colors=np.empty((0, 3), dtype=np.uint8),
                radii=np.empty((0,), dtype=np.float32),
            ),
        )

    rov_centers, rov_colors, rov_half_sizes = _rovlar_to_rerun_arrays(getattr(app, "rovs", None))
    if rov_centers.shape[0] == 0:
        rr.log(
            "simulasyon/rovlar",
            rr.Boxes3D(
                centers=np.empty((0, 3), dtype=np.float32),
                half_sizes=np.empty((0, 3), dtype=np.float32),
                colors=np.empty((0, 3), dtype=np.uint8),
            ),
        )
    else:
        rr.log(
            "simulasyon/rovlar",
            rr.Boxes3D(
                centers=rov_centers,
                half_sizes=rov_half_sizes,
                colors=rov_colors,
            ),
        )
