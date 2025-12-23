"""
100 Senaryo Üretme Scripti

Bu script, senaryo modülünü kullanarak 100 farklı senaryo üretir.
Her senaryo için farklı parametreler kullanılır ve veri toplanabilir.
"""

from FiratROVNet import senaryo
import numpy as np
import random
import time

def senaryo_uret_100():
    """
    100 farklı senaryo üretir ve her birinden veri toplar.
    """
    print("=" * 60)
    print("100 Senaryo Üretme Başlatılıyor...")
    print("=" * 60)
    
    # Senaryo verilerini saklamak için liste
    tum_senaryolar = []
    
    # İstatistikler
    toplam_rov_sayisi = 0
    toplam_engel_sayisi = 0
    
    # 100 senaryo üret
    for senaryo_no in range(1, 101):
        print(f"\n[{senaryo_no}/100] Senaryo oluşturuluyor...")
        
        # Her senaryo için farklı parametreler
        n_rovs = random.randint(3, 8)  # 3-8 arası ROV
        n_engels = random.randint(10, 30)  # 10-30 arası engel
        havuz_genisligi = random.choice([150, 200, 250])  # Farklı havuz boyutları
        
        # Engel tipleri (rastgele karışım)
        engel_tipleri = []
        for _ in range(n_engels):
            if random.random() < 0.8:  # %80 kaya
                engel_tipleri.append('kaya')
            else:  # %20 ağaç
                engel_tipleri.append('agac')
        
        try:
            # Senaryo oluştur
            senaryo.uret(
                n_rovs=n_rovs,
                n_engels=n_engels,
                havuz_genisligi=havuz_genisligi,
                engel_tipleri=engel_tipleri
            )
            
            # Senaryo bilgilerini topla
            senaryo_bilgisi = {
                'senaryo_no': senaryo_no,
                'n_rovs': n_rovs,
                'n_engels': n_engels,
                'havuz_genisligi': havuz_genisligi,
                'rov_verileri': []
            }
            
            # Her ROV için veri topla
            for rov_id in range(n_rovs):
                try:
                    batarya = senaryo.get(rov_id, "batarya")
                    gps = senaryo.get(rov_id, "gps")
                    hiz = senaryo.get(rov_id, "hiz")
                    sonar = senaryo.get(rov_id, "sonar")
                    rol = senaryo.get(rov_id, "rol")
                    engel_mesafesi = senaryo.get(rov_id, "engel_mesafesi")
                    iletisim_menzili = senaryo.get(rov_id, "iletisim_menzili")
                    
                    senaryo_bilgisi['rov_verileri'].append({
                        'rov_id': rov_id,
                        'batarya': batarya,
                        'gps': gps.tolist() if isinstance(gps, np.ndarray) else gps,
                        'hiz': hiz.tolist() if isinstance(hiz, np.ndarray) else hiz,
                        'sonar': sonar,
                        'rol': rol,
                        'engel_mesafesi': engel_mesafesi,
                        'iletisim_menzili': iletisim_menzili
                    })
                except Exception as e:
                    print(f"  ⚠️ ROV-{rov_id} verisi alınamadı: {e}")
            
            # Senaryo bilgisini kaydet
            tum_senaryolar.append(senaryo_bilgisi)
            
            # İstatistikleri güncelle
            toplam_rov_sayisi += n_rovs
            toplam_engel_sayisi += n_engels
            
            # İlerleme göster
            if senaryo_no % 10 == 0:
                print(f"  ✅ {senaryo_no} senaryo tamamlandı")
                print(f"     Ortalama ROV sayısı: {toplam_rov_sayisi / senaryo_no:.1f}")
                print(f"     Ortalama engel sayısı: {toplam_engel_sayisi / senaryo_no:.1f}")
            
            # Senaryoyu temizle (bellek yönetimi için)
            senaryo.temizle()
            
            # Küçük bir gecikme (sistem kaynaklarını korumak için)
            time.sleep(0.01)
            
        except Exception as e:
            print(f"  ❌ Senaryo {senaryo_no} oluşturulamadı: {e}")
            # Hata durumunda da temizle
            try:
                senaryo.temizle()
            except:
                pass
            continue
    
    # Özet rapor
    print("\n" + "=" * 60)
    print("ÖZET RAPOR")
    print("=" * 60)
    print(f"✅ Toplam {len(tum_senaryolar)} senaryo başarıyla oluşturuldu")
    print(f"📊 Toplam ROV sayısı: {toplam_rov_sayisi}")
    print(f"📊 Toplam engel sayısı: {toplam_engel_sayisi}")
    print(f"📊 Ortalama ROV sayısı: {toplam_rov_sayisi / len(tum_senaryolar):.2f}")
    print(f"📊 Ortalama engel sayısı: {toplam_engel_sayisi / len(tum_senaryolar):.2f}")
    
    # İlk 3 senaryonun detaylarını göster
    print("\n" + "=" * 60)
    print("İLK 3 SENARYO DETAYLARI")
    print("=" * 60)
    for i, sen in enumerate(tum_senaryolar[:3]):
        print(f"\nSenaryo {sen['senaryo_no']}:")
        print(f"  ROV sayısı: {sen['n_rovs']}")
        print(f"  Engel sayısı: {sen['n_engels']}")
        print(f"  Havuz genişliği: {sen['havuz_genisligi']}")
        print(f"  ROV verileri: {len(sen['rov_verileri'])} ROV")
        if sen['rov_verileri']:
            print(f"    ROV-0 Batarya: {sen['rov_verileri'][0]['batarya']:.2f}")
            print(f"    ROV-0 GPS: {sen['rov_verileri'][0]['gps']}")
    
    return tum_senaryolar


def senaryo_uret_ve_simule_et(n_simulasyon_adimi=10):
    """
    100 senaryo üretir ve her birinde simülasyon adımları çalıştırır.
    
    Args:
        n_simulasyon_adimi: Her senaryoda kaç adım simülasyon yapılacak
    """
    print("=" * 60)
    print(f"100 Senaryo Üretme ve Simülasyon ({n_simulasyon_adimi} adım/senaryo)")
    print("=" * 60)
    
    tum_senaryolar = []
    
    for senaryo_no in range(1, 101):
        print(f"\n[{senaryo_no}/100] Senaryo oluşturuluyor ve simüle ediliyor...")
        
        n_rovs = random.randint(3, 8)
        n_engels = random.randint(10, 30)
        havuz_genisligi = random.choice([150, 200, 250])
        
        try:
            # Senaryo oluştur
            senaryo.uret(
                n_rovs=n_rovs,
                n_engels=n_engels,
                havuz_genisligi=havuz_genisligi
            )
            
            # Simülasyon adımları
            sim_verileri = []
            for adim in range(n_simulasyon_adimi):
                # Simülasyonu güncelle
                senaryo.guncelle(0.016)  # 16ms (60 FPS)
                
                # Her adımda veri topla
                adim_verisi = {
                    'adim': adim,
                    'rov_verileri': []
                }
                
                for rov_id in range(n_rovs):
                    try:
                        gps = senaryo.get(rov_id, "gps")
                        hiz = senaryo.get(rov_id, "hiz")
                        batarya = senaryo.get(rov_id, "batarya")
                        
                        adim_verisi['rov_verileri'].append({
                            'rov_id': rov_id,
                            'gps': gps.tolist() if isinstance(gps, np.ndarray) else gps,
                            'hiz': hiz.tolist() if isinstance(hiz, np.ndarray) else hiz,
                            'batarya': batarya
                        })
                    except Exception as e:
                        pass
                
                sim_verileri.append(adim_verisi)
            
            # Senaryo bilgisini kaydet
            tum_senaryolar.append({
                'senaryo_no': senaryo_no,
                'n_rovs': n_rovs,
                'n_engels': n_engels,
                'havuz_genisligi': havuz_genisligi,
                'simulasyon_verileri': sim_verileri
            })
            
            if senaryo_no % 10 == 0:
                print(f"  ✅ {senaryo_no} senaryo tamamlandı")
            
            # Temizle
            senaryo.temizle()
            time.sleep(0.01)
            
        except Exception as e:
            print(f"  ❌ Senaryo {senaryo_no} hatası: {e}")
            try:
                senaryo.temizle()
            except:
                pass
            continue
    
    print(f"\n✅ Toplam {len(tum_senaryolar)} senaryo simüle edildi")
    return tum_senaryolar


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='100 Senaryo Üretme Scripti')
    parser.add_argument('--simule', action='store_true', 
                       help='Senaryoları simüle et (varsayılan: sadece oluştur)')
    parser.add_argument('--adim', type=int, default=10,
                       help='Simülasyon adım sayısı (varsayılan: 10)')
    
    args = parser.parse_args()
    
    if args.simule:
        # Simülasyonlu mod
        sonuclar = senaryo_uret_ve_simule_et(n_simulasyon_adimi=args.adim)
    else:
        # Sadece senaryo oluşturma modu
        sonuclar = senaryo_uret_100()
    
    print("\n✅ Tüm işlemler tamamlandı!")


