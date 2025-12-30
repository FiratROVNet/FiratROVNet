"""
GAT Eğitimi İçin 100 Veri Üretimi Test Scripti
===============================================

Bu script, GAT eğitimi için 100 adet veri üretir ve istatistiklerini gösterir.

Kullanım:
    python senaryo_uret_100.py
"""

import sys
import os

# Proje kök dizinini path'e ekle
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from FiratROVNet.gat import GATVeriUretici
import time

def main():
    print("=" * 60)
    print("GAT Eğitimi İçin 100 Veri Üretimi Testi")
    print("=" * 60)
    print()
    
    # GAT veri üreticisini oluştur
    uretici = GATVeriUretici()
    
    # 100 veri üret
    print("📊 100 adet veri üretiliyor...")
    print()
    
    baslangic_zamani = time.time()
    veri_listesi = []
    
    for i in range(200):
        try:
            data = uretici.veri_uret()
            veri_listesi.append(data)
            
            # İlerleme göster
            if (i + 1) % 10 == 0:
                gecen_sure = time.time() - baslangic_zamani
                ortalama_sure = gecen_sure / (i + 1)
                kalan_sure = ortalama_sure * (100 - (i + 1))
                print(f"   ✅ {i + 1}/100 tamamlandı | "
                      f"Ortalama: {ortalama_sure:.2f}s | "
                      f"Tahmini kalan: {kalan_sure:.1f}s")
        except Exception as e:
            print(f"   ❌ Veri {i + 1} üretilirken hata: {e}")
            import traceback
            traceback.print_exc()
            break
    
    toplam_sure = time.time() - baslangic_zamani
    
    print()
    print("=" * 60)
    print("✅ Veri Üretimi Tamamlandı!")
    print("=" * 60)
    print(f"   Toplam veri sayısı: {len(veri_listesi)}")
    print(f"   Toplam süre: {toplam_sure:.2f} saniye")
    print(f"   Ortalama süre/veri: {toplam_sure / len(veri_listesi):.3f} saniye")
    print()
    
    # İstatistikler
    print("📈 İstatistikler hesaplanıyor...")
    print()
    
    istatistikler = uretici.istatistikler(n_samples=100)
    
    print("=" * 60)
    print("İSTATİSTİKLER")
    print("=" * 60)
    print(f"   Toplam örnek: {istatistikler['toplam_ornek']}")
    print(f"   Toplam ROV: {istatistikler['toplam_rov']}")
    print(f"   Ortalama ROV/örnek: {istatistikler['ortalama_rov_per_ornek']:.2f}")
    print(f"   Toplam edge: {istatistikler['toplam_edge']}")
    print(f"   Ortalama edge/örnek: {istatistikler['ortalama_edge_per_ornek']:.2f}")
    print()
    print("GAT Kodları Dağılımı:")
    print(f"   OK (0):      {istatistikler['gat_kodlari_sayilari'][0]:4d} "
          f"({istatistikler['gat_kodlari_dagilimi'][0]*100:.1f}%)")
    print(f"   ENGEL (1):   {istatistikler['gat_kodlari_sayilari'][1]:4d} "
          f"({istatistikler['gat_kodlari_dagilimi'][1]*100:.1f}%)")
    print(f"   CARPISMA (2): {istatistikler['gat_kodlari_sayilari'][2]:4d} "
          f"({istatistikler['gat_kodlari_dagilimi'][2]*100:.1f}%)")
    print(f"   KOPUK (3):   {istatistikler['gat_kodlari_sayilari'][3]:4d} "
          f"({istatistikler['gat_kodlari_dagilimi'][3]*100:.1f}%)")
    print(f"   UZAK (4):    {istatistikler['gat_kodlari_sayilari'][4]:4d} "
          f"({istatistikler['gat_kodlari_dagilimi'][4]*100:.1f}%)")
    print()
    print("=" * 60)
    print("✅ Test Tamamlandı!")
    print("=" * 60)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️ Test kullanıcı tarafından iptal edildi.")
    except Exception as e:
        print(f"\n\n❌ Test sırasında hata oluştu: {e}")
        import traceback
        traceback.print_exc()
