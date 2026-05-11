"""
Konsol'dan tek satırda çalıştır:
    exec(open('gorev_baslat.py').read())
"""

# --- GRUP 0: Sol alan ---
filo.alan_tarama_baslat(grup_id=0, alan=(-150, 0, 0, 200), derinlik=-20.0, sessiz=False)

# --- GRUP 1: Sağ alan ---
filo.alan_tarama_baslat(grup_id=1, alan=(0, 0, 150, 200), derinlik=-20.0, sessiz=False)

# --- Sadece grup liderlerine YOLO aç ---
for rov in filo.g_rovs.get(0, []) + filo.g_rovs.get(1, []):
    if getattr(rov, 'role', 0) == 1:
        filo.camera_manager.kamera_ekle(rov_id=rov.id)
        filo.yolo_baslat(rov.id, model_path="yolov8n.pt")

print("✅ Görevler başlatıldı. Tespitler: filo.camera_manager.yolo_son_tespitler")
