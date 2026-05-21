"""
Simulasyon konsolundan tek satirda calistir:
    exec(open("gorev_baslat.py", encoding="utf-8").read())
"""


def gorevleri_baslat(filo):
    """Iki alan tarama gorevini baslatir ve grup liderlerinde YOLO'yu acar."""
    # --- GRUP 0: Sol alan ---
    filo.alan_tarama_baslat(grup_id=0, alan=(-150, 0, 0, 200), derinlik=-20.0, sessiz=False)

    # --- GRUP 1: Sag alan ---
    filo.alan_tarama_baslat(grup_id=1, alan=(0, 0, 150, 200), derinlik=-20.0, sessiz=False)

    # --- Sadece grup liderlerine YOLO ac ---
    for rov in list(filo.g_rovs.get(0, [])) + list(filo.g_rovs.get(1, [])):
        if getattr(rov, "role", 0) == 1:
            filo.camera_manager.kamera_ekle(rov_id=rov.id)
            filo.yolo_baslat(rov.id, model_path="yolov8n.pt")

    print("Görevler başlatıldı. Tespitler: filo.camera_manager.yolo_son_tespitler")


def _konsol_filo_al():
    runtime_filo = globals().get("filo")
    if runtime_filo is None:
        raise RuntimeError("gorev_baslat.py simülasyon konsolunda 'filo' globali ile çalıştırılmalı.")
    return runtime_filo


if __name__ == "__main__":
    gorevleri_baslat(_konsol_filo_al())
