"""
Senaryo Modülü Kullanım Örnekleri

Bu dosya, senaryo.py modülünün nasıl kullanılacağını gösterir.
"""

import senaryo
import numpy as np

# ==========================================
# ÖRNEK 1: Basit Senaryo Oluşturma
# ==========================================

print("=" * 60)
print("ÖRNEK 1: Basit Senaryo")
print("=" * 60)

# Senaryo oluştur
senaryo.uret(n_rovs=4, n_engels=20, havuz_genisligi=200)

# Veri al
batarya = senaryo.get(0, "batarya")
gps = senaryo.get(0, "gps")
sonar = senaryo.get(0, "sonar")

print(f"ROV-0 Batarya: {batarya}")
print(f"ROV-0 GPS: {gps}")
print(f"ROV-0 Sonar: {sonar}")

# Filo üzerinden erişim
if senaryo.filo:
    batarya2 = senaryo.filo.get(0, "batarya")
    print(f"Filo üzerinden ROV-0 Batarya: {batarya2}")

# Temizle
senaryo.temizle()

# ==========================================
# ÖRNEK 2: Özel Engel Tipleri
# ==========================================

print("\n" + "=" * 60)
print("ÖRNEK 2: Özel Engel Tipleri")
print("=" * 60)

# Farklı engel tipleri
engel_tipleri = ['kaya'] * 10 + ['agac'] * 5

senaryo.uret(
    n_rovs=3,
    n_engels=15,
    havuz_genisligi=150,
    engel_tipleri=engel_tipleri
)

# Tüm ROV'ların batarya durumunu kontrol et
for i in range(3):
    batarya = senaryo.get(i, "batarya")
    gps = senaryo.get(i, "gps")
    print(f"ROV-{i}: Batarya={batarya:.2f}, GPS={gps}")

senaryo.temizle()

# ==========================================
# ÖRNEK 3: Özel Başlangıç Pozisyonları
# ==========================================

print("\n" + "=" * 60)
print("ÖRNEK 3: Özel Başlangıç Pozisyonları")
print("=" * 60)

baslangic_pozisyonlari = {
    0: (0, -5, 0),      # ROV-0 merkez
    1: (10, -5, 10),    # ROV-1 sağ-ileri
    2: (-10, -5, -10),  # ROV-2 sol-geri
}

senaryo.uret(
    n_rovs=3,
    n_engels=15,
    baslangic_pozisyonlari=baslangic_pozisyonlari
)

# Pozisyonları kontrol et
for i in range(3):
    gps = senaryo.get(i, "gps")
    print(f"ROV-{i} Başlangıç Pozisyonu: {gps}")

senaryo.temizle()

# ==========================================
# ÖRNEK 4: Simülasyon Adımları
# ==========================================

print("\n" + "=" * 60)
print("ÖRNEK 4: Simülasyon Adımları")
print("=" * 60)

senaryo.uret(n_rovs=2, n_engels=10)

# İlk durum
print("Başlangıç:")
gps1 = senaryo.get(0, "gps")
print(f"ROV-0 GPS: {gps1}")

# Hedef ata
senaryo.git(0, 50, 60, -10)

# Birkaç adım simüle et
for adim in range(10):
    senaryo.guncelle(0.016)  # 16ms (60 FPS)
    
    if adim % 3 == 0:  # Her 3 adımda bir yazdır
        gps = senaryo.get(0, "gps")
        hiz = senaryo.get(0, "hiz")
        print(f"Adım {adim}: GPS={gps}, Hız={hiz}")

senaryo.temizle()

# ==========================================
# ÖRNEK 5: Sensör Ayarları
# ==========================================

print("\n" + "=" * 60)
print("ÖRNEK 5: Özel Sensör Ayarları")
print("=" * 60)

sensor_ayarlari = {
    'lider': {
        'engel_mesafesi': 30.0,
        'iletisim_menzili': 50.0
    },
    'takipci': {
        'engel_mesafesi': 20.0,
        'iletisim_menzili': 40.0
    }
}

senaryo.uret(
    n_rovs=3,
    n_engels=15,
    sensor_ayarlari=sensor_ayarlari
)

# Sensör ayarlarını kontrol et
for i in range(3):
    engel_mesafesi = senaryo.get(i, "engel_mesafesi")
    iletisim_menzili = senaryo.get(i, "iletisim_menzili")
    rol = senaryo.get(i, "rol")
    print(f"ROV-{i} (Rol: {rol}): Engel Mesafesi={engel_mesafesi}, İletişim Menzili={iletisim_menzili}")

senaryo.temizle()

# ==========================================
# ÖRNEK 6: Veri Toplama (AI Eğitimi İçin)
# ==========================================

print("\n" + "=" * 60)
print("ÖRNEK 6: Veri Toplama (AI Eğitimi)")
print("=" * 60)

senaryo.uret(n_rovs=4, n_engels=20)

# Veri seti oluştur
veri_seti = []

for adim in range(100):  # 100 adım veri topla
    senaryo.guncelle(0.016)
    
    # Her ROV için veri topla
    adim_verisi = {}
    for rov_id in range(4):
        adim_verisi[rov_id] = {
            'gps': senaryo.get(rov_id, "gps"),
            'hiz': senaryo.get(rov_id, "hiz"),
            'batarya': senaryo.get(rov_id, "batarya"),
            'sonar': senaryo.get(rov_id, "sonar"),
            'rol': senaryo.get(rov_id, "rol")
        }
    
    veri_seti.append(adim_verisi)
    
    if adim % 20 == 0:
        print(f"Adım {adim}: {len(veri_seti)} veri noktası toplandı")

print(f"\n✅ Toplam {len(veri_seti)} adım veri toplandı")
print(f"   Her adımda {len(veri_seti[0])} ROV verisi")

# İlk adımın verisini göster
print("\nİlk adım verisi:")
for rov_id, veri in veri_seti[0].items():
    print(f"  ROV-{rov_id}: {veri}")

senaryo.temizle()

print("\n✅ Tüm örnekler tamamlandı!")
