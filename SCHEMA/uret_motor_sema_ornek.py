"""
Örnek: Motor şemasını SCHEMA/ROV{id}/ içinde üretmek için (simülasyon olmadan).
Proje kökünden: python SCHEMA/uret_motor_sema_ornek.py
"""

import json
import os
import sys

# SCHEMA klasörünü path'e ekle (standalone plot_motor_schema için)
SCHEMA_DIR = os.path.dirname(os.path.abspath(__file__))
if SCHEMA_DIR not in sys.path:
    sys.path.insert(0, SCHEMA_DIR)
from plot_motor_schema import draw_rov_motor_schema

# __init__.py ile aynı motor layout (konum + Euler açıları derece)
MOTOR_ORNEK = [
    {"name": "m0", "position": (-200.0, 0.0, 200.0), "rotation": (90, 0.0, -45)},
    {"name": "m1", "position": (200.0, 0.0, 200.0),  "rotation": (90, 0.0, 45)},
    {"name": "m2", "position": (-200.0, 0.0, -200.0), "rotation": (-90, 0.0, 135)},
    {"name": "m3", "position": (200.0, 0.0, -200.0),  "rotation": (-90, 0.0, -135)},
    {"name": "m4", "position": (-100, 0.0, 0.0),     "rotation": (0.0, 0, 0.0)},
    {"name": "m5", "position": (100, 0.0, 0.0),      "rotation": (0.0, 0, 0.0)},
]

def save_rov_schema_info(rov_id, motor_entries, save_dir):
    os.makedirs(save_dir, exist_ok=True)
    data = {
        "rov_id": rov_id,
        "motorlar": [{"name": e["name"], "position": list(e["position"]), "rotation": list(e["rotation"])} for e in motor_entries],
    }
    with open(os.path.join(save_dir, "bilgi.json"), "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

if __name__ == "__main__":
    rov_id = 0  # Örnek için ROV0
    save_dir = os.path.join(SCHEMA_DIR, f"ROV{rov_id}")
    save_rov_schema_info(rov_id, MOTOR_ORNEK, save_dir)
    paths = draw_rov_motor_schema(MOTOR_ORNEK, save_dir=save_dir, base_name="rov_motor_sema")
    print("Oluşturulan PDF (SCHEMA/ROV{}):".format(rov_id), paths.get("pdf"))
