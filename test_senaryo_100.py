"""
Senaryo Modülü Test - 100 Örnek Üretme

Bu script, senaryo modülünü kullanarak 100 tane simülasyon örneği üretir
ve ROV sensör verilerini toplar.

Features:
- Dynamic obstacle support (engel_bulutu)
- Depth preservation across navigation
- A* pathfinding with dynamic obstacles
"""

import os
import sys
import json
import time
import numpy as np
from pathlib import Path

# Repo root ekle
REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from FiratROVNet import senaryo

def test_senaryo_100_ornekler():
    """
    Senaryo modülünü kullanarak 100 tane örnek üret.
    """
    print("="*70)
    print("🚀 SENARYO MODÜLÜ - 100 ÖRNEK ÜRETIM TESTİ")
    print("="*70)
    
    n_ornekler = 1000
    
    # Veri depolama
    ornekler = []
    basarili = 0
    basarisiz = 0
    
    start_time = time.time()
    
    print(f"\n📊 Ortam Ayarları:")
    print(f"   - Örnek Sayısı: {n_ornekler}")
    print(f"   - ROV Sayısı: RASTGELE (4-12)")
    print(f"   - Ada Sayısı: RASTGELE (3-6)")
    print(f"   - Kaya Sayısı: RASTGELE (10-20)")
    print(f"\n🔄 Örnek üretimi başlıyor...\n")
    
    for ornekNo in range(1, n_ornekler + 1):
        try:
            # 1. Senaryo oluştur - RASTGELE sayıda entity (headless)
            # Parametresiz çağırılırsa object pooling otomatik rastgele seçer:
            # ROV: 4-12, Ada: 3-6, Kaya: 10-20
            senaryo.uret(
                havuz_genisligi=200,
                verbose=False
            )
            
            if not senaryo.filo:
                print(f"❌ Örnek {ornekNo}: Filo kurulmadı")
                basarisiz += 1
                continue
                basarisiz += 1
                continue
            
            # 2. Aktif ROV sayısını tespit et (görünür olanlar)
            aktif_rovlar = [r for r in senaryo.ortam.rovs if r and hasattr(r, 'visible') and r.visible]
            n_rovs = len(aktif_rovlar)
            
            # YENİ: Grup sayısını random belirle (1-4) ve her ROV'a random grup ata
            num_groups = np.random.randint(1, 5)  # 1, 2, 3 veya 4
            group_ids = list(range(num_groups))  # [0], [0,1], [0,1,2] veya [0,1,2,3]
            
            for rov_id in range(n_rovs):
                rov = senaryo.ortam.rovs[rov_id]
                if rov:
                    rov.group_id = np.random.choice(group_ids)  # Random grup ata
            
            # Rastgele hedefler ata ve verileri topla
            ornek_verisi = {
                "ornekNo": ornekNo,
                "rovlar": []
            }
            
            for rov_id in range(n_rovs):
                try:
                    # Rastgele hedef
                    hedef_x = np.random.uniform(-150, 150)
                    hedef_z = np.random.uniform(-150, 150)
                    hedef_y = np.random.uniform(-30, 0)  # Derinlik (-35m'ye kadar batabilir)
                    
                    # Hedef ata (A* pathfinding + dinamik engel desteği)
                    senaryo.git(rov_id, hedef_x, hedef_z, hedef_y, ai=True)
                    
                    # 2.5. Sensörleri güncelle (Sadece bu ROV - headless modda raycast manuel gerekli)
                    rov = senaryo.ortam.rovs[rov_id] if rov_id < len(senaryo.ortam.rovs) else None
                    if rov and hasattr(rov, '_guncelle_sensorler'):
                        try:
                            rov._guncelle_sensorler()
                        except:
                            pass
                    
                    # 2.6. GPS Sinyal'i güncelle (Derinliğe göre: <-5m = sinyal yok)
                    if rov and hasattr(rov, 'gnc'):
                        try:
                            gps = senaryo.get(rov_id, "gps")
                            if gps and gps[2] < -5.0:
                                rov.gnc.gps_sinyal = 0
                            else:
                                rov.gnc.gps_sinyal = 1
                        except:
                            pass
                    
                    # 3. ROV verisini topla (genişletilmiş: lidar, group_id, mod, gps_sinyal)
                    gps = senaryo.get(rov_id, "gps")
                    batarya = senaryo.get(rov_id, "batarya")
                    hiz = senaryo.get(rov_id, "hiz")
                    sonar = senaryo.get(rov_id, "sonar")
                    lidar = senaryo.get(rov_id, "lidar")  # Lidar verileri
                    rol = senaryo.get(rov_id, "rol")
                    group_id = senaryo.get(rov_id, "group_id")  # Grup ID
                    mod = senaryo.get(rov_id, "mod")  # YENİ: GNC mod
                    gps_sinyal = senaryo.get(rov_id, "gps_sinyal")  # YENİ: GPS sinyal durumu
                    
                    # Veri tamlığını kontrol et
                    if gps is None or batarya is None:
                        print(f"⚠️ Örnek {ornekNo}/ROV-{rov_id}: Veri alınamadı (gps={gps}, batarya={batarya})")
                        continue
                except Exception as e:
                    print(f"⚠️ Örnek {ornekNo}/ROV-{rov_id}: Veri toplama hatası: {e}")
                    import traceback
                    traceback.print_exc()
                    continue
                
                rov_veri = {
                    "rov_id": rov_id,
                    "group_id": int(group_id) if group_id is not None else 0,
                    "hedef": {
                        "x": float(hedef_x),
                        "z": float(hedef_z),
                        "y": float(hedef_y)
                    },
                    "nerede": {
                        "x": float(gps[0]) if hasattr(gps, '__getitem__') else float(gps.x) if hasattr(gps, 'x') else 0.0,
                        "y": float(gps[1]) if hasattr(gps, '__getitem__') else float(gps.y) if hasattr(gps, 'y') else 0.0,
                        "z": float(gps[2]) if hasattr(gps, '__getitem__') else float(gps.z) if hasattr(gps, 'z') else 0.0,
                    },
                    "batarya": float(batarya) if batarya is not None else 0.0,
                    "rol": int(rol) if rol is not None else 0,
                    "sonar": float(sonar) if sonar is not None else 999.0,
                    "mod": int(mod) if mod is not None else 0,  # YENİ
                    "gps_sinyal": int(gps_sinyal) if gps_sinyal is not None else 0,  # YENİ
                    "lidar": {}
                }
                
                # Lidar verilerini ekle
                if lidar is not None and isinstance(lidar, dict):
                    for lidar_id, mesafe in lidar.items():
                        rov_veri["lidar"][str(lidar_id)] = float(mesafe) if mesafe is not None else -1.0
                
                # Derinlik koruması kontrolü
                if rov_veri["nerede"]["z"] is not None and hedef_y is not None:
                    derinlik_fark = abs(rov_veri["nerede"]["z"] - hedef_y)
                    rov_veri["derinlik_koruması"] = "✅" if derinlik_fark < 5 else "⚠️"
                
                ornek_verisi["rovlar"].append(rov_veri)
            
            if ornek_verisi["rovlar"]:
                ornekler.append(ornek_verisi)
                basarili += 1
                
                # Progress indicator
                if ornekNo % 10 == 0:
                    print(f"✅ {ornekNo}/100 örnek üretildi")
            else:
                basarisiz += 1
            
            # 4. Scenario'yu temizle
            senaryo.temizle()
            
        except Exception as e:
            hata_mesaji = str(e)
            print(f"❌ Örnek {ornekNo}: {hata_mesaji}")
            # İlk 3 hata için tam traceback göster
            if basarisiz < 3:
                import traceback
                traceback.print_exc()
            basarisiz += 1
            try:
                senaryo.temizle()
            except:
                pass
    
    elapsed_time = time.time() - start_time
    
    # Sonuçları göster
    print(f"\n{'='*70}")
    print(f"✅ SONUÇLAR")
    print(f"{'='*70}")
    print(f"Toplam Örnek: {n_ornekler}")
    print(f"✅ Başarılı: {basarili}")
    print(f"❌ Başarısız: {basarisiz}")
    print(f"⏱️  Toplam Süre: {elapsed_time:.2f}s")
    print(f"📊 Ortalama/Örnek: {elapsed_time/n_ornekler:.3f}s")
    
    # Örnek veriler göster
    if ornekler:
        print(f"\n{'='*70}")
        print(f"📋 İLK 4 ÖRNEK:")
        print(f"{'='*70}")
        
        for i, ornek in enumerate(ornekler[:4]):
            print(f"\n🔹 Örnek {ornek['ornekNo']}:")
            print(f"   ROV Sayısı: {len(ornek['rovlar'])}")
            
            for rov in ornek['rovlar']:
                hedef = rov['hedef']
                nerede = rov['nerede']
                
                # Basitleştirilmiş format - Entity property'leri
                print(f"\n   📍 ROV:")
                print(f"      id: {rov['rov_id']}")
                print(f"      group_id: {rov['group_id']}")
                print(f"      gps: ({nerede['x']:.1f}, {nerede['y']:.1f}, {nerede['z']:.1f})")
                print(f"      hedef: ({hedef['x']:.1f}, {hedef['z']:.1f}, {hedef['y']:.1f})")
                print(f"      rol: {rov['rol']}")
                print(f"      mod: {rov.get('mod', 0)}")
                print(f"      gps_sinyal: {rov.get('gps_sinyal', 0)}")
                print(f"      sonar: {rov['sonar']:.1f}")
                print(f"      batarya: {rov['batarya']:.1%}")
                
                # Lidar dict formatında
                if 'lidar' in rov and rov['lidar']:
                    lidar_dict = {int(k): float(v) for k, v in rov['lidar'].items()}
                    print(f"      lidar: {lidar_dict}")
    
    # JSON dosyasına kaydet
    output_file = os.path.join(REPO_ROOT, "ornekler_100.json")
    with open(output_file, 'w') as f:
        json.dump({
            "toplam": n_ornekler,
            "basarili": basarili,
            "basarisiz": basarisiz,
            "ornekler": ornekler[:10]  # İlk 10 örneği kaydet (dosya boyutu için)
        }, f, indent=2)
    
    print(f"\n💾 Veri kaydedildi: {output_file}")
    
    # Özet istatistikler
    if ornekler:
        print(f"\n{'='*70}")
        print(f"📊 İSTATİSTİKLER")
        print(f"{'='*70}")
        
        # Batarya istatistikleri
        tum_bataryalar = []
        for ornek in ornekler:
            for rov in ornek['rovlar']:
                tum_bataryalar.append(rov['batarya'])
        
        if tum_bataryalar:
            print(f"🔋 Batarya:")
            print(f"   Min:  {np.min(tum_bataryalar):.1%}")
            print(f"   Max:  {np.max(tum_bataryalar):.1%}")
            print(f"   Ort:  {np.mean(tum_bataryalar):.1%}")
        
        # Sensör istatistikleri (>= 0 ise engel algılandı)
        sonar_ile_engel_algilayan_rovlar = 0
        lidar_ile_engel_algilayan_rovlar = 0
        tum_sonarlar = []
        tum_lidarlar = []
        gps_sinyal_0_sayisi = 0
        gps_sinyal_1_sayisi = 0
        
        for ornek in ornekler:
            for rov in ornek['rovlar']:
                # Sonar kontrolü
                if rov['sonar'] >= 0:
                    tum_sonarlar.append(rov['sonar'])
                    sonar_ile_engel_algilayan_rovlar += 1
                
                # Lidar kontrolü (herhangi bir kanalda engel varsa)
                if 'lidar' in rov and rov['lidar']:
                    lidar_engel_var = False
                    for lidar_id, mesafe in rov['lidar'].items():
                        if mesafe >= 0:
                            tum_lidarlar.append(mesafe)
                            lidar_engel_var = True
                    if lidar_engel_var:
                        lidar_ile_engel_algilayan_rovlar += 1
                
                # GPS sinyal kontrolü
                if rov.get('gps_sinyal', 0) == 0:
                    gps_sinyal_0_sayisi += 1
                else:
                    gps_sinyal_1_sayisi += 1
        
        # Sonar istatistikleri
        if sonar_ile_engel_algilayan_rovlar > 0:
            print(f"📡 Sonar İstatistikleri:")
            print(f"   🔴 Engel Algılayan ROV Sayısı: {sonar_ile_engel_algilayan_rovlar}")
            print(f"   Mesafe - Min:  {np.min(tum_sonarlar):.1f}m")
            print(f"   Mesafe - Max:  {np.max(tum_sonarlar):.1f}m")
            print(f"   Mesafe - Ort:  {np.mean(tum_sonarlar):.1f}m")
        else:
            print(f"📡 Sonar: Engel algılanmadı")
        
        # Lidar istatistikleri
        if lidar_ile_engel_algilayan_rovlar > 0:
            print(f"🔦 Lidar İstatistikleri:")
            print(f"   🔴 Engel Algılayan ROV Sayısı: {lidar_ile_engel_algilayan_rovlar}")
            print(f"   Mesafe - Min:  {np.min(tum_lidarlar):.1f}m")
            print(f"   Mesafe - Max:  {np.max(tum_lidarlar):.1f}m")
            print(f"   Mesafe - Ort:  {np.mean(tum_lidarlar):.1f}m")
        else:
            print(f"🔦 Lidar: Engel algılanmadı")
        
        # GPS Sinyal istatistikleri
        print(f"📡 GPS Sinyal İstatistikleri:")
        print(f"   🔴 GPS Şimdi LosT (gps_sinyal=0): {gps_sinyal_0_sayisi}")
        print(f"   ✅ GPS Aktif (gps_sinyal=1): {gps_sinyal_1_sayisi}")
    
    print(f"\n✅ TEST TAMAMLANDI!\n")
    
    return basarili, basarisiz

if __name__ == "__main__":
    basarili, basarisiz = test_senaryo_100_ornekler()
    
    # Exit code
    exit_code = 0 if basarisiz == 0 else 1
    sys.exit(exit_code)
