import json
import math
import os
import numpy as np
from FiratROVNet.config import HavuzAyarlari

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import matplotlib.transforms as mtransforms
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

def _euler_deg_to_direction(rx_deg, ry_deg, rz_deg):
    """
    Kritik Düzeltme: Rotasyonlar tam olarak belirtilen sırayla (X -> Z -> Y) uygulanır.
    rx: 90 ve -90 arasındaki farkı korumak için başlangıç vektörü (0,1,0) dikey seçilir.
    """

    #print(rx_deg, ry_deg, rz_deg)
    rx, ry, rz = map(math.radians, [rx_deg, ry_deg, rz_deg])
    
    # Başlangıç: Motor dik duruyor (Azure durumu)
    v = np.array([0, 1, 0])

    # 1. X ekseninde yatır (Pitch) - rx: 90 ileri (+Z), -90 geri (-Z) yapar.
    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(rx), -math.sin(rx)],
        [0, math.sin(rx), math.cos(rx)]
    ])
    
    # 2. Z ekseninde döndür (Roll/Açı)
    Ry = np.array([
        [math.cos(rz),0, math.sin(rz)],
        [0, 1, 0],
        [-math.sin(rz),0, math.cos(rz)]

    ])
    
    # 3. Y ekseninde döndür (Yaw)
    Rz = np.array([
        [math.cos(ry), -math.sin(ry),0],
        [math.sin(ry), math.cos(ry),0],
        [0, 0, 1]
    ])

    # Dönüşüm Zinciri: Ry * Rx * Rz * v
    # Bu sıra, 90 derece yatmış bir motorun rz açısıyla yer düzleminde dönmesini sağlar.
    res = Ry @ (Rz @ (Rx @ v))
    return res[0], res[1], res[2]

def save_rov_schema_info(rov_id, motor_entries, save_dir):
    os.makedirs(save_dir, exist_ok=True)
    data = {"rov_id": rov_id, "motorlar": motor_entries}
    with open(os.path.join(save_dir, "bilgi.json"), "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

def draw_rov_motor_schema(motor_entries, save_dir, world_pos=(0,0,0), world_rot=(0,0,0), pool_size=None, base_name="rov_motor_sema"):
    if not HAS_MPL: raise RuntimeError("matplotlib gerekli")
    if pool_size is None:
        pool_size = (HavuzAyarlari.HAVUZ_TAM_GENISLIK, HavuzAyarlari.HAVUZ_TAM_GENISLIK)
    os.makedirs(save_dir, exist_ok=True)

    # AYARLAR (Sabit Ok Boyutları)
    map_arrow_scale = 15.0  # Harita üzerindeki okların sabit uzunluğu
    body_half_z = 120
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 36))

    # --- 1. PANEL: ÜST GÖRÜNÜM (YEREL) ---
    ax1.set_aspect("equal")
    ax1.set_xlim(-300, 300); ax1.set_ylim(-300, 300)
    ax1.set_title("1. Detaylı Motor Konfigürasyonu (P, Y, R Bilgili)", fontsize=14)
    ax1.add_patch(mpatches.Rectangle((-100, -body_half_z), 200, 2*body_half_z, color="gray", alpha=0.1))
    ax1.add_patch(mpatches.Polygon([(-35, body_half_z), (35, body_half_z), (0, body_half_z+40)], color="green", alpha=0.5))

    # --- 2. PANEL: YAN GÖRÜNÜM ---
    ax2.set_aspect("equal")
    ax2.set_xlim(-300, 300); ax2.set_ylim(-150, 150)
    ax2.set_title("2. Yan Profil İtki Analizi", fontsize=14)
    ax2.add_patch(mpatches.Rectangle((-body_half_z, -50), 2*body_half_z, 100, color="gray", alpha=0.1))

    # --- 3. PANEL: SAHA HARİTASI ---
    p_w, p_l = pool_size
    ax3.set_aspect("equal")
    ax3.set_xlim(-p_w/2, p_w/2); ax3.set_ylim(-p_l/2, p_l/2)
    ax3.set_title(f"3. Saha Operasyon Haritası (Havuz: {p_w}x{p_l}m)", fontsize=14)
    ax3.grid(True, linestyle='-', alpha=0.2, color="blue")
    ax3.add_patch(mpatches.Rectangle((-p_w/2, -p_l/2), p_w, p_l, fill=False, edgecolor="blue", linewidth=4, alpha=0.5))

    # ROV Global Transformasyonu
    w_x, w_y, w_z = world_pos
    w_yaw = world_rot[1]
    mpl_angle = -w_yaw # Rotasyon yönü eşleme
    tr = mtransforms.Affine2D().translate(-w_x, -w_z).rotate_deg(mpl_angle).translate(w_x, w_z)
    t = tr + ax3.transData
    
    # ROV Gövdesi ve Burun
    icon_w, icon_h = 50, 60
    ax3.add_patch(mpatches.Rectangle((w_x-icon_w/2, w_z-icon_h/2), icon_w, icon_h, fill=True, color="red", alpha=0.1, transform=t))
    ax3.add_patch(mpatches.Rectangle((w_x-icon_w/2, w_z-icon_h/2), icon_w, icon_h, fill=False, edgecolor="red", linewidth=2, transform=t))
    ax3.add_patch(mpatches.Polygon([[w_x, w_z+icon_h/2+25], [w_x-20, w_z+icon_h/2-5], [w_x+20, w_z+icon_h/2-5]], color="green", alpha=0.8, transform=t))

    for e in motor_entries:
        xm_l, ym_l, zm_l = e["position"]; rx, ry, rz = e["rotation"]
        dx, dy, dz = _euler_deg_to_direction(rx, ry, rz)
        
        # --- ORTAK ÇİZİM MANTIĞI ---
        # Panel 1 (Yerel)
        if abs(dy) > 0.8:
            ax1.add_patch(mpatches.Circle((xm_l, zm_l), 22, color="midnightblue", fill=False))
            if dy > 0: ax1.plot(xm_l, zm_l, 'o', color="midnightblue", markersize=8)
            else: ax1.text(xm_l, zm_l, 'X', color="midnightblue", fontsize=10, fontweight='bold', ha='center', va='center')
        else:
            ax1.arrow(xm_l, zm_l, dx*50, dz*50, head_width=20, head_length=15, fc="midnightblue", length_includes_head=True)
        ax1.annotate(f"{e['name']}\n({int(rx)},{int(ry)},{int(rz)})", (xm_l, zm_l), xytext=(20, 20), textcoords="offset points", fontsize=8, bbox=dict(boxstyle="round", fc="white", alpha=0.7))

        # Panel 2 (Yan)
        ax2.plot(zm_l, ym_l, "o", color="steelblue")
        ax2.arrow(zm_l, ym_l, dz*50, dy*50, head_width=20, head_length=15, fc="forestgreen", length_includes_head=True)

        # Panel 3 (SAHA HARİTASI - Sabit Boyutlu Oklar)
        s = 0.15 # İkon ölçeği
        curr_x, curr_z = xm_l * s, zm_l * s
        
        if abs(dy) > 0.8: # Dikey
            circ = mpatches.Circle((w_x+curr_x, w_z+curr_z), radius=6, color="midnightblue", fill=False, transform=t)
            ax3.add_patch(circ)
            if dy > 0: ax3.plot(w_x+curr_x, w_z+curr_z, 'o', color="midnightblue", markersize=2, transform=t)
            else: ax3.text(w_x+curr_x, w_z+curr_z, 'x', color="midnightblue", fontsize=7, ha='center', va='center', transform=t)
        else: # Yatay (Normalize edilmiş sabit ok boyutu)
            mag = math.sqrt(dx**2 + dz**2)
            if mag > 0:
                ndx, ndz = (dx/mag), (dz/mag)
                ax3.arrow(w_x+curr_x, w_z+curr_z, ndx*map_arrow_scale, ndz*map_arrow_scale, 
                          head_width=6, head_length=6, fc="midnightblue", ec="black", length_includes_head=True, transform=t)

        ax3.text(w_x+curr_x+8, w_z+curr_z+8, f"{e['name']}\n({int(rx)},{int(ry)},{int(rz)})", 
                 fontsize=5, fontweight='bold', transform=t, bbox=dict(boxstyle="round,pad=0.1", fc="white", alpha=0.6))

    # --- BİLGİ KUTUSU (BURUNA ORTALANMIŞ) ---
    rad = math.radians(mpl_angle + 90)
    bx, bz = w_x + math.cos(rad)*65, w_z + math.sin(rad)*65
    info = f"X:{w_x:.1f} Z:{w_z:.1f} Y:{w_y:.1f}m | Yaw:{w_yaw:.1f}°"
    ax3.text(bx, bz, info, fontsize=8, fontweight='bold', ha='center', va='center', bbox=dict(boxstyle="round,pad=0.3", fc="yellow", alpha=0.8))

    plt.tight_layout()
    fig.savefig(os.path.join(save_dir, f"{base_name}.pdf"), bbox_inches="tight")
    plt.close(fig)
    return {"pdf": os.path.join(save_dir, f"{base_name}.pdf")}