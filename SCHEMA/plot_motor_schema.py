"""
Bağımsız ROV motor şeması çizimi (sadece matplotlib + math).
Simülasyon (ursina) olmadan çalıştırmak için SCHEMA klasöründe kullanılır.
"""

import math
import os

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


def _euler_deg_to_direction(rx_deg, ry_deg, rz_deg):
    """Euler derece (x,y,z) -> birim yön (x,y,z)."""
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy_ = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    x = cx * sz + sx * sy_ * cz
    y = cx * cz - sx * sy_ * sz
    z = -sx * cy
    n = math.sqrt(x * x + y * y + z * z) or 1e-9
    return (x / n, y / n, z / n)


def draw_rov_motor_schema(motor_entries, save_dir, base_name="rov_motor_sema"):
    """
    Teknik çizim mantığıyla tek PDF: üstte üst görünüm, altta yan görünüm.
    motor_entries: [{"name": "m0", "position": (x,y,z), "rotation": (rx,ry,rz)}, ...]
    """
    if not HAS_MPL:
        raise RuntimeError("matplotlib gerekli: pip install matplotlib")
    os.makedirs(save_dir, exist_ok=True)
    all_x = [e["position"][0] for e in motor_entries]
    all_y = [e["position"][1] for e in motor_entries]
    all_z = [e["position"][2] for e in motor_entries]
    margin = 80
    x_min, x_max = min(all_x) - margin, max(all_x) + margin
    y_min, y_max = min(all_y) - margin, max(all_y) + margin
    z_min, z_max = min(all_z) - margin, max(all_z) + margin
    body_half_x = max(abs(x) for x in all_x) * 0.6
    body_half_z = max(abs(z) for z in all_z) * 0.6
    arrow_scale = 120.0

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))

    # Üst görünüm (XZ)
    ax1.set_aspect("equal")
    ax1.set_xlim(z_min, z_max)
    ax1.set_ylim(x_min, x_max)
    ax1.set_xlabel("Z (ön/arka)", fontsize=11)
    ax1.set_ylabel("X (sol/sağ)", fontsize=11)
    ax1.set_title("Üst görünüm (XZ)", fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.axhline(0, color="k", linewidth=0.5)
    ax1.axvline(0, color="k", linewidth=0.5)
    rect = mpatches.Rectangle(
        (-body_half_z, -body_half_x), 2 * body_half_z, 2 * body_half_x,
        linewidth=1.5, edgecolor="gray", facecolor="lightgray", alpha=0.6
    )
    ax1.add_patch(rect)
    tri_len = body_half_z * 0.5
    tri_w = body_half_x * 0.5
    on_ucgen_ust = mpatches.Polygon(
        [(body_half_z + tri_len, 0), (body_half_z - tri_len, -tri_w), (body_half_z - tri_len, tri_w)],
        closed=True, facecolor="darkgreen", edgecolor="black", linewidth=1.5, alpha=0.9, zorder=5
    )
    ax1.add_patch(on_ucgen_ust)
    ax1.plot(0, 0, "k+", markersize=10, label="Merkez")
    for e in motor_entries:
        name = e["name"]
        x, y, z = e["position"][0], e["position"][1], e["position"][2]
        rot = e["rotation"]
        rx = rot[0] if len(rot) > 0 else 0
        ry = rot[1] if len(rot) > 1 else 0
        rz = rot[2] if len(rot) > 2 else 0
        dx, dy, dz = _euler_deg_to_direction(rx, ry, rz)
        ax1.plot(z, x, "o", color="steelblue", markersize=12)
        ax1.arrow(z, x, dz * arrow_scale, dx * arrow_scale,
                  head_width=15, head_length=12, fc="darkblue", ec="darkblue")
        ax1.annotate(f"{name}\n({rx:.0f},{ry:.0f},{rz:.0f})°",
                    (z, x), textcoords="offset points", xytext=(8, 8), fontsize=8,
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
    ax1.legend(loc="upper right", fontsize=8)

    # Yan görünüm (ZY)
    ax2.set_aspect("equal")
    ax2.set_xlim(z_min, z_max)
    ax2.set_ylim(y_min, y_max)
    ax2.set_xlabel("Z (ön/arka)", fontsize=11)
    ax2.set_ylabel("Y (aşağı/yukarı)", fontsize=11)
    ax2.set_title("Yan görünüm (ZY)", fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.axhline(0, color="k", linewidth=0.5)
    ax2.axvline(0, color="k", linewidth=0.5)
    rect2 = mpatches.Rectangle((-body_half_z, -50), 2 * body_half_z, 100,
                                linewidth=1.5, edgecolor="gray", facecolor="lightgray", alpha=0.6)
    ax2.add_patch(rect2)
    tri_len2 = body_half_z * 0.5
    tri_w2 = 45.0
    on_ucgen_yan = mpatches.Polygon(
        [(body_half_z + tri_len2, 0), (body_half_z - tri_len2, -tri_w2), (body_half_z - tri_len2, tri_w2)],
        closed=True, facecolor="darkgreen", edgecolor="black", linewidth=1.5, alpha=0.9, zorder=5
    )
    ax2.add_patch(on_ucgen_yan)
    ax2.plot(0, 0, "k+", markersize=10)
    for e in motor_entries:
        name = e["name"]
        x, y, z = e["position"][0], e["position"][1], e["position"][2]
        rot = e["rotation"]
        rx = rot[0] if len(rot) > 0 else 0
        ry = rot[1] if len(rot) > 1 else 0
        rz = rot[2] if len(rot) > 2 else 0
        dx, dy, dz = _euler_deg_to_direction(rx, ry, rz)
        ax2.plot(z, y, "o", color="steelblue", markersize=12)
        ax2.arrow(z, y, dz * arrow_scale, dy * arrow_scale,
                  head_width=15, head_length=12, fc="darkgreen", ec="darkgreen")
        ax2.annotate(f"{name}\n({rx:.0f},{ry:.0f},{rz:.0f})°",
                    (z, y), textcoords="offset points", xytext=(8, 8), fontsize=8,
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
    plt.suptitle("ROV Motor Şeması", fontsize=14, y=1.02)
    plt.tight_layout()
    pdf_path = os.path.join(save_dir, f"{base_name}.pdf")
    fig.savefig(pdf_path, bbox_inches="tight")
    plt.close(fig)
    return {"pdf": pdf_path}
