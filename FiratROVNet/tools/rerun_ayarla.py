import math
import os
import socket
import threading
from io import StringIO
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import quote, urlsplit, urlunsplit

import numpy as np
import rerun as rr

from FiratROVNet.config import PerformansAyarlari

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


def _port_adaylari(port, deneme_sayisi=5):
    base = int(port)
    return [base + i for i in range(max(1, int(deneme_sayisi)))]


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


def rerun_baslat(ip_adresi=None, kayit_dosyasi=None):
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

    server_uri = ""
    aktif_grpc_port = rr_grpc_port
    grpc_hatasi = None
    for port in _port_adaylari(rr_grpc_port):
        try:
            server_uri = str(rr.serve_grpc(grpc_port=port))
            aktif_grpc_port = port
            break
        except Exception as e:
            grpc_hatasi = e
            print(f"[Rerun] gRPC port {port} baslatilamadi: {e}")

    if not server_uri:
        print(f"[Rerun] gRPC sunucusu baslatilamadi: {grpc_hatasi}")
        return {
            "lan_ip": lan_ip,
            "server_uri": "",
            "server_uri_lan": "",
            "web_local_url": "",
            "web_lan_url": "",
            "alias_local_url": "",
            "alias_lan_url": "",
            "alias_server": None,
        }
    server_uri_lan = _uri_host_degistir(server_uri, lan_ip)

    if kayit_dosyasi:
        print("[Rerun] Uyari: canli web viewer baslangicinda eszamanli dosya kaydi desteklenmiyor; kayit butonunu kullan.")

    web_viewer_aktif = False
    aktif_web_port = rr_web_port
    web_hatasi = None
    for port in _port_adaylari(rr_web_port):
        try:
            rr.serve_web_viewer(
                web_port=port,
                connect_to=server_uri_lan,
                open_browser=open_browser,
            )
            aktif_web_port = port
            web_viewer_aktif = True
            break
        except Exception as e:
            web_hatasi = e
            print(f"[Rerun] Web viewer port {port} baslatilamadi: {e}")

    if not web_viewer_aktif:
        print(f"[Rerun] Web viewer baslatilamadi: {web_hatasi}")

    web_local_url = f"http://127.0.0.1:{aktif_web_port}/?url={quote(server_uri, safe='')}"
    web_lan_url = f"http://{lan_ip}:{aktif_web_port}/?url={quote(server_uri_lan, safe='')}"
    alias_local_url = f"http://127.0.0.1:{rr_alias_port}{rr_alias_path}"
    alias_lan_url = f"http://{lan_ip}:{rr_alias_port}{rr_alias_path}"

    alias_server = None
    if not web_viewer_aktif:
        print(f"[Rerun] gRPC: {server_uri_lan}")
    elif rr_alias_port == rr_web_port:
        print("[Rerun] RR_ALIAS_PORT ve RR_WEB_PORT ayni oldugu icin alias route acilamadi.")
        print("[Rerun] Farkli port ver: RR_WEB_PORT=9091 RR_ALIAS_PORT=9090")
    else:
        try:
            alias_server = _start_rr_alias_server("0.0.0.0", rr_alias_port, rr_alias_path, web_lan_url)
            print(f"[Rerun] Alias: {alias_lan_url}")
        except Exception as e:
            print(f"[Rerun] Alias sunucusu baslatilamadi: {e}")

    if web_viewer_aktif:
        print(f"[Rerun] Web: {web_lan_url}")

    return {
        "lan_ip": lan_ip,
        "server_uri": server_uri,
        "server_uri_lan": server_uri_lan,
        "grpc_port": aktif_grpc_port,
        "web_port": aktif_web_port,
        "web_local_url": web_local_url,
        "web_lan_url": web_lan_url,
        "alias_local_url": alias_local_url,
        "alias_lan_url": alias_lan_url,
        "alias_server": alias_server,
    }


def rerun_kayit_baslat(runtime, kayit_dosyasi):
    """Canli Rerun viewer devam ederken dosya kaydini baslatir."""
    server_uri = str((runtime or {}).get("server_uri", ""))
    if not server_uri:
        print("[Rerun] Kayit baslatilamadi: server_uri yok.")
        return False
    try:
        rr.set_sinks(rr.GrpcSink(url=server_uri), rr.FileSink(kayit_dosyasi))
        print(f"[Rerun] Kayit basladi: {kayit_dosyasi}")
        return True
    except Exception as e:
        print(f"[Rerun] Kayit baslatma hatasi: {e}")
        return False


def rerun_kayit_durdur(runtime):
    """Dosya kaydini bitirir, canli Rerun viewer sink'ini korur."""
    server_uri = str((runtime or {}).get("server_uri", ""))
    if not server_uri:
        print("[Rerun] Kayit durdurulamadi: server_uri yok.")
        return False
    try:
        rr.set_sinks(rr.GrpcSink(url=server_uri))
        print("[Rerun] Kayit durduruldu.")
        return True
    except Exception as e:
        print(f"[Rerun] Kayit durdurma hatasi: {e}")
        return False


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


def _rr_downsample_np(arr, max_count):
    if arr is None:
        return arr
    try:
        n = int(arr.shape[0])
    except Exception:
        return arr
    max_count = int(max_count or 0)
    if max_count <= 0 or n <= max_count:
        return arr
    idx = np.linspace(0, n - 1, max_count, dtype=np.int64)
    return arr[idx]


# ---------------------------------------------------------------------------
# Multibeam sonar 3D zemin haritalama
# ---------------------------------------------------------------------------

# Bathymetrik renk duraklari (derin → sığ)
# [0.0] derin = koyu mavi | [0.4] orta = teal | [0.7] sığ = yeşil | [1.0] yüzey = sarı
_BATI_STOPS = np.array([
    [0,   10, 100],   # derin mavi
    [0,  150, 180],   # teal
    [60, 200,  60],   # yeşil
    [220, 220,  0],   # sarı
], dtype=np.float32)
_BATI_T = np.array([0.0, 0.20, 0.45, 75], dtype=np.float32)


def _batimetri_renkleri_hesapla(ursina_y_dizi, sea_floor_y, surface_y):
    """Ursina Y yükseklik değerlerine göre bathymetrik renk hesapla."""
    if ursina_y_dizi is None or len(ursina_y_dizi) == 0:
        return np.empty((0, 3), dtype=np.uint8)
    yuk = np.asarray(ursina_y_dizi, dtype=np.float32)
    dip = float(sea_floor_y)
    yuzey = float(surface_y)
    if yuzey <= dip:
        oran = np.ones_like(yuk, dtype=np.float32)
    else:
        oran = np.clip((yuk - dip) / (yuzey - dip), 0.0, 1.0)
    # Dört duraklı doğrusal interpolasyon
    renkler = np.zeros((len(oran), 3), dtype=np.float32)
    for i in range(len(_BATI_T) - 1):
        t0, t1 = _BATI_T[i], _BATI_T[i + 1]
        mask = (oran >= t0) & (oran <= t1)
        if not np.any(mask):
            continue
        alfa = ((oran[mask] - t0) / (t1 - t0)).reshape(-1, 1)
        renkler[mask] = _BATI_STOPS[i] * (1 - alfa) + _BATI_STOPS[i + 1] * alfa
    return np.clip(np.rint(renkler), 0, 255).astype(np.uint8)


def _sonar_footprint_noktalari_hesapla(rov, sea_floor_y, swath_acisi_deg,
                                       nokta_sayisi, along_track_sayi, gurultu_sigma):
    """
    Tek bir ROV için multibeam sonar footprint noktaları üret.

    Model:
    - ROV'un altına bakan sonar konisi, across-track yönünde ±swath_acisi_deg
      açısıyla zemine çarpar.
    - Along-track'te küçük bir şerit de eklenir (örtüşme/doku için).
    - Zemin noktalarına Gaussian gürültü eklenerek gerçekçi deniz tabanı dokusu
      simüle edilir.

    Döndürür: (N×3 float32) Ursina uzayı (x, y_zemin, z) veya None
    """
    try:
        ux = float(rov.position.x)
        uy = float(rov.position.y)
        uz = float(rov.position.z)
    except (AttributeError, TypeError, ValueError):
        return None

    altitude = uy - float(sea_floor_y)
    if altitude < 0.5:
        return None

    swath_rad = math.radians(float(swath_acisi_deg))
    half_width = altitude * math.tan(swath_rad)

    yaw_deg = float(getattr(rov, 'rotation_y', 0.0))
    yaw_rad = math.radians(yaw_deg)

    # Across-track yönü (swath genişliği boyunca)
    acx = math.cos(yaw_rad)
    acz = -math.sin(yaw_rad)
    # Along-track yönü (hareket yönü)
    alx = math.sin(yaw_rad)
    alz = math.cos(yaw_rad)

    # Along-track spread = half_width'in %20'si (ince şerit = gerçekçi süpürme)
    along_spread = half_width * 0.20
    t_values = np.linspace(-half_width, half_width, int(nokta_sayisi))
    s_values = np.linspace(-along_spread, along_spread, int(along_track_sayi))

    rng = np.random.default_rng()
    n = len(t_values) * len(s_values)
    noise = rng.normal(0.0, float(gurultu_sigma), n).astype(np.float32)

    points = np.empty((n, 3), dtype=np.float32)
    idx = 0
    floor_y = float(sea_floor_y)
    for s in s_values:
        for t in t_values:
            points[idx, 0] = ux + t * acx + s * alx   # Ursina X
            points[idx, 1] = floor_y + noise[idx]      # Ursina Y (zemin + gürültü)
            points[idx, 2] = uz + t * acz + s * alz    # Ursina Z
            idx += 1
    return points


def _sonar_haritasi_guncelle(app):
    """
    Tüm aktif ROV'ların sonar footprint noktalarını hesaplayıp
    app.rerun_tarama_haritasi tamponuna ekler.
    Yalnızca ROV MIN_HAREKET_ESIGI kadar hareket etmişse yeni nokta üretir.
    """
    try:
        from FiratROVNet.config import SonarHaritalamaAyarlari
    except ImportError:
        return

    harita: list = app.rerun_tarama_haritasi
    son_poz: dict = app._rr_son_tarama_poz
    sea_floor_y = float(getattr(app, 'SEA_FLOOR_Y', -50.0))
    esik = float(SonarHaritalamaAyarlari.MIN_HAREKET_ESIGI)

    for rov in getattr(app, 'rovs', []) or []:
        if rov is None or getattr(rov, 'is_destroyed', False):
            continue
        try:
            rov_id = int(getattr(rov, 'rov_id', id(rov)))
            ux = float(rov.position.x)
            uy = float(rov.position.y)
            uz = float(rov.position.z)
        except (AttributeError, TypeError, ValueError):
            continue

        prev = son_poz.get(rov_id)
        if prev is not None:
            dx, dy, dz = ux - prev[0], uy - prev[1], uz - prev[2]
            if math.sqrt(dx * dx + dy * dy + dz * dz) < esik:
                continue

        yeni = _sonar_footprint_noktalari_hesapla(
            rov, sea_floor_y,
            SonarHaritalamaAyarlari.SWATH_ACISI_DERECE,
            SonarHaritalamaAyarlari.NOKTA_SAYISI,
            SonarHaritalamaAyarlari.ALONG_TRACK_SAYI,
            SonarHaritalamaAyarlari.GURULTU_SIGMA,
        )
        if yeni is None or len(yeni) == 0:
            continue

        harita.extend(yeni.tolist())
        son_poz[rov_id] = (ux, uy, uz)

    # Sliding-window: eski noktaları at
    max_nokta = SonarHaritalamaAyarlari.MAKSIMUM_NOKTA
    fazla = len(harita) - max_nokta
    if fazla > 0:
        del harita[:fazla]


def _rovlar_to_rerun_arrays(rovs):
    centers = []
    colors = []

    for rov in rovs or []:
        if rov is None or getattr(rov, "is_destroyed", False):
            continue
        try:
            is_empty = getattr(rov, "is_empty", None)
            if callable(is_empty) and is_empty():
                continue
            centers.append(_ursina_to_rerun_xyz(rov.position.x, rov.position.y, rov.position.z))
        except (AssertionError, AttributeError, TypeError, ValueError):
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
    # Rerun icin kırpılmayan tam gecmis tampon tercih edilir; yoksa normal bulut kullanilir
    engel_kaynak = getattr(app, "rerun_engel_bulutu", None)
    if not isinstance(engel_kaynak, list) or len(engel_kaynak) == 0:
        engel_kaynak = getattr(app, "engel_bulutu", None)
    engel_points = _engel_bulutu_to_points3d(engel_kaynak)
    engel_points = _rr_downsample_np(engel_points, getattr(PerformansAyarlari, "RERUN_MAKS_ENGEL_NOKTASI", 2500))
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

    statik_tekrar = int(getattr(PerformansAyarlari, "RERUN_STATIK_LOG_TEKRAR_ADIMI", 300) or 0)
    statik_son_adim = getattr(app, "_rr_statik_son_adim", None)
    statik_logla = statik_son_adim is None or (statik_tekrar > 0 and int(step) - int(statik_son_adim) >= statik_tekrar)
    if statik_logla:
        app._rr_statik_son_adim = int(step)
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
            ada_cevre_np = _rr_downsample_np(ada_cevre_np, getattr(PerformansAyarlari, "RERUN_MAKS_ENGEL_NOKTASI", 2500))
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

    # ------------------------------------------------------------------
    # Multibeam sonar 3D zemin haritalama (bathymetrik point cloud)
    # ------------------------------------------------------------------
    try:
        from FiratROVNet.config import SonarHaritalamaAyarlari
        tarama_raw = getattr(app, 'rerun_tarama_haritasi', [])
        tarama_adimi = max(1, int(getattr(PerformansAyarlari, "RERUN_TARAMA_LOG_ADIMI", 5) or 1))
        try:
            tarama_len = len(tarama_raw) if tarama_raw is not None else 0
        except TypeError:
            tarama_len = 0
        tarama_logla = tarama_len > 0 and (
            int(step) % tarama_adimi == 0 or tarama_len != int(getattr(app, "_rr_son_tarama_len", -1))
        )
        if tarama_logla:
            app._rr_son_tarama_len = tarama_len
            tarama_np = np.asarray(tarama_raw, dtype=np.float32)
            tarama_np = _rr_downsample_np(tarama_np, getattr(PerformansAyarlari, "RERUN_MAKS_TARAMA_NOKTASI", 12000))
            # Ursina (x, y, z) → Rerun (x, z, y) koordinat dönüşümü
            tarama_rr = tarama_np[:, [0, 2, 1]]
            tarama_renkler = _batimetri_renkleri_hesapla(
                tarama_np[:, 1], sea_floor_y=sea_floor_y, surface_y=surface_y
            )
            tarama_radii = np.full(
                (tarama_rr.shape[0],),
                SonarHaritalamaAyarlari.NOKTA_RADIUS,
                dtype=np.float32,
            )
            rr.log("tarama/zemin_haritasi", rr.Points3D(tarama_rr, colors=tarama_renkler, radii=tarama_radii))
        elif tarama_len == 0 and getattr(app, "_rr_son_tarama_len", None) != 0:
            app._rr_son_tarama_len = 0
            rr.log(
                "tarama/zemin_haritasi",
                rr.Points3D(
                    np.empty((0, 3), dtype=np.float32),
                    colors=np.empty((0, 3), dtype=np.uint8),
                    radii=np.empty((0,), dtype=np.float32),
                ),
            )
    except Exception:
        pass
