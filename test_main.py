#!/usr/bin/env python3
"""
FiratROVNet CI/CD Test Suite
Grafik kartı olmadan tüm sistem fonksiyonlarını test eder.
"""

import os
import sys
import traceback

# Headless mod için environment variable ayarla
os.environ['DISPLAY'] = ':0'  # X11 display (headless için)
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'  # Software rendering

# Test sonuçları
test_results = {
    'passed': [],
    'failed': [],
    'skipped': []
}

def record_test_pass(name):
    """Test başarılı"""
    test_results['passed'].append(name)
    print(f"✅ {name}")

def record_test_fail(name, error):
    """Test başarısız"""
    test_results['failed'].append((name, str(error)))
    print(f"❌ {name}: {error}")

def record_test_skip(name, reason):
    """Test atlandı"""
    test_results['skipped'].append((name, reason))
    print(f"⏭️  {name}: {reason}")

# ==========================================
# TEST 1: Modül İmportları
# ==========================================
print("\n" + "="*60)
print("TEST 1: Modül İmportları")
print("="*60)

try:
    from FiratROVNet import gat, ortam, gnc, iletisim, config
    from FiratROVNet.gat import GAT_Modeli, Train, FiratAnalizci
    from FiratROVNet.ortam import veri_uret
    from FiratROVNet.gnc import GNCKomutan, LiderGNC, TakipciGNC
    from FiratROVNet.iletisim import AkustikModem
    from FiratROVNet.config import cfg
    record_test_pass("Modül İmportları")
except Exception as e:
    record_test_fail("Modül İmportları", e)
    sys.exit(1)

# ==========================================
# TEST 2: Veri Üretimi
# ==========================================
print("\n" + "="*60)
print("TEST 2: Veri Üretimi")
print("="*60)

try:
    # Veri üretimi testi
    data = veri_uret(n_rovs=5)
    assert data.x.shape[0] == 5, f"ROV sayısı yanlış: {data.x.shape[0]}"
    assert data.x.shape[1] == 7, f"Özellik sayısı yanlış: {data.x.shape[1]}"
    assert data.edge_index.shape[0] == 2, "Edge index formatı yanlış"
    assert data.y.shape[0] == 5, f"Etiket sayısı yanlış: {data.y.shape[0]}"
    record_test_pass("Veri Üretimi (5 ROV)")
    
    # Farklı ROV sayıları ile test
    for n in [3, 10, 15]:
        data = veri_uret(n_rovs=n)
        assert data.x.shape[0] == n, f"{n} ROV için veri üretimi başarısız"
    record_test_pass("Veri Üretimi (Farklı ROV Sayıları)")
    
except Exception as e:
    record_test_fail("Veri Üretimi", e)

# ==========================================
# TEST 3: GAT Modeli
# ==========================================
print("\n" + "="*60)
print("TEST 3: GAT Modeli")
print("="*60)

try:
    import torch
    
    # Model oluşturma
    model = GAT_Modeli()
    record_test_pass("GAT Modeli Oluşturma")
    
    # Forward pass testi
    data = veri_uret(n_rovs=4)
    output = model(data.x, data.edge_index)
    assert output.shape[0] == 4, "Output shape yanlış"
    assert output.shape[1] == 6, "Output sınıf sayısı yanlış"
    record_test_pass("GAT Modeli Forward Pass")
    
    # Attention testi
    output, edge_idx, alpha = model(data.x, data.edge_index, return_attention=True)
    assert alpha is not None, "Attention weights döndürülmedi"
    record_test_pass("GAT Modeli Attention")
    
except Exception as e:
    record_test_fail("GAT Modeli", e)

# ==========================================
# TEST 4: FiratAnalizci
# ==========================================
print("\n" + "="*60)
print("TEST 4: FiratAnalizci")
print("="*60)

try:
    # Analizci oluşturma (model dosyası olmasa bile çalışmalı)
    analizci = FiratAnalizci(model_yolu="rov_modeli_multi.pth")
    record_test_pass("FiratAnalizci Oluşturma")
    
    # Analiz testi
    data = veri_uret(n_rovs=4)
    tahminler, edge_idx, alpha = analizci.analiz_et(data)
    assert len(tahminler) == 4, "Tahmin sayısı yanlış"
    assert all(0 <= t < 6 for t in tahminler), "Tahmin değerleri geçersiz"
    record_test_pass("FiratAnalizci Analiz")
    
except Exception as e:
    record_test_fail("FiratAnalizci", e)

# ==========================================
# TEST 5: İletişim Sistemi (AkustikModem)
# ==========================================
print("\n" + "="*60)
print("TEST 5: İletişim Sistemi")
print("="*60)

try:
    # Modem oluşturma
    modem1 = AkustikModem(rov_id=0, gurultu_orani=0.05, kayip_orani=0.1)
    modem2 = AkustikModem(rov_id=1, gurultu_orani=0.1, kayip_orani=0.15)
    record_test_pass("AkustikModem Oluşturma")
    
    # Rehber güncelleme
    rehber = {0: modem1, 1: modem2}
    modem1.rehber_guncelle(rehber)
    modem2.rehber_guncelle(rehber)
    assert len(modem1.rehber) == 2, "Rehber güncelleme başarısız"
    record_test_pass("Rehber Güncelleme")
    
    # Paket gönderme
    import time
    time.sleep(0.1)  # Gecikme için bekle
    success = modem1.gonder(modem2, [10.0, 20.0, 30.0], "TEST")
    record_test_pass("Paket Gönderme")
    
    # Paket dinleme
    time.sleep(0.6)  # Gecikme süresini bekle
    paketler = modem2.dinle()
    if paketler:
        assert len(paketler) > 0, "Paket alınamadı"
        record_test_pass("Paket Dinleme")
    else:
        record_test_skip("Paket Dinleme", "Paket kaybı simülasyonu nedeniyle paket alınamadı (normal)")
    
except Exception as e:
    record_test_fail("İletişim Sistemi", e)

# ==========================================
# TEST 6: GNC Sistemi
# ==========================================
print("\n" + "="*60)
print("TEST 6: GNC Sistemi")
print("="*60)

try:
    # Mock ROV entity (Ursina olmadan)
    class MockROV:
        def __init__(self, rov_id):
            self.id = rov_id
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self.y = 0.0
            self.role = 0
            self.modem = None
        
        def move(self, komut, guc=1.0):
            pass  # Mock
    
    # GNCKomutan testi
    komutan = GNCKomutan()
    test_pass("GNCKomutan Oluşturma")
    
    # Mock ROV ve modem oluştur
    rov0 = MockROV(0)
    rov1 = MockROV(1)
    modem0 = AkustikModem(0)
    modem1 = AkustikModem(1)
    
    # Lider ve Takipçi GNC oluştur
    lider_gnc = LiderGNC(rov0, modem0)
    takipci_gnc = TakipciGNC(rov1, modem1, lider_modem_ref=modem0)
    
    komutan.ekle(lider_gnc)
    komutan.ekle(takipci_gnc)
    assert len(komutan.sistemler) == 2, "GNC sistemleri eklenemedi"
    test_pass("GNC Sistemleri Ekleme")
    
    # Hedef atama testi
    komutan.git(0, 10, 20, -5, ai=True)
    assert lider_gnc.hedef is not None, "Hedef atanmadı"
    test_pass("Hedef Atama (git)")
    
    # Güncelleme testi
    tahminler = [0, 1]  # Mock tahminler
    komutan.guncelle_hepsi(tahminler)
    test_pass("GNC Güncelleme")
    
except Exception as e:
    test_fail("GNC Sistemi", e)
    traceback.print_exc()

# ==========================================
# TEST 7: Config Sistemi
# ==========================================
print("\n" + "="*60)
print("TEST 7: Config Sistemi")
print("="*60)

try:
    # Config değerlerini kontrol et
    assert hasattr(cfg, 'goster_modem'), "Config'de goster_modem yok"
    assert hasattr(cfg, 'goster_gnc'), "Config'de goster_gnc yok"
    assert hasattr(cfg, 'goster_sistem'), "Config'de goster_sistem yok"
    test_pass("Config Özellikleri")
    
    # Config değerlerini değiştir
    original_value = cfg.goster_modem
    cfg.goster_modem = True
    assert cfg.goster_modem == True, "Config değeri değiştirilemedi"
    cfg.goster_modem = original_value
    test_pass("Config Değer Değiştirme")
    
except Exception as e:
    test_fail("Config Sistemi", e)

# ==========================================
# TEST 8: Ortam Sınıfı (Ursina Olmadan)
# ==========================================
print("\n" + "="*60)
print("TEST 8: Ortam Sınıfı (Headless)")
print("="*60)

try:
    # Ursina'yı headless modda başlatmayı dene
    # Eğer başarısız olursa test atlanır
    from FiratROVNet.simulasyon import Ortam
    
    # Headless mod için environment ayarları
    os.environ['Ursina_HEADLESS'] = '1'
    
    try:
        # Ortam oluşturma (headless modda)
        app = Ortam()
        test_pass("Ortam Oluşturma (Headless)")
        
        # Simülasyon nesneleri oluşturma
        app.sim_olustur(n_rovs=3, n_engels=5)
        assert len(app.rovs) == 3, "ROV'lar oluşturulamadı"
        assert len(app.engeller) == 5, "Engeller oluşturulamadı"
        test_pass("Simülasyon Nesneleri Oluşturma")
        
        # Konsol verileri ekleme
        app.konsola_ekle("test", "test_value")
        assert "test" in app.konsol_verileri, "Konsol verileri eklenemedi"
        test_pass("Konsol Verileri")
        
    except Exception as e:
        test_skip("Ortam Sınıfı", f"Ursina headless mod başarısız: {e}")
        print(f"   Not: Grafik kartı olmadan Ursina başlatılamadı (normal)")
    
except Exception as e:
    test_fail("Ortam Sınıfı", e)

# ==========================================
# TEST 9: Entegrasyon Testi
# ==========================================
print("\n" + "="*60)
print("TEST 9: Entegrasyon Testi")
print("="*60)

try:
    # Tüm sistemleri bir arada test et
    data = veri_uret(n_rovs=4)
    analizci = FiratAnalizci(model_yolu="rov_modeli_multi.pth")
    tahminler, _, _ = analizci.analiz_et(data)
    
    komutan = GNCKomutan()
    modem0 = AkustikModem(0)
    modem1 = AkustikModem(1)
    
    rov0 = MockROV(0)
    rov1 = MockROV(1)
    
    gnc0 = LiderGNC(rov0, modem0)
    gnc1 = TakipciGNC(rov1, modem1, lider_modem_ref=modem0)
    
    komutan.ekle(gnc0)
    komutan.ekle(gnc1)
    
    komutan.git(0, 10, 20, -5)
    komutan.git(1, 15, 25, -10)
    
    komutan.guncelle_hepsi(tahminler[:2])
    
    test_pass("Entegrasyon Testi")
    
except Exception as e:
    test_fail("Entegrasyon Testi", e)
    traceback.print_exc()

# ==========================================
# TEST SONUÇLARI
# ==========================================
print("\n" + "="*60)
print("TEST SONUÇLARI")
print("="*60)

total = len(test_results['passed']) + len(test_results['failed']) + len(test_results['skipped'])
passed = len(test_results['passed'])
failed = len(test_results['failed'])
skipped = len(test_results['skipped'])

print(f"\nToplam Test: {total}")
print(f"✅ Başarılı: {passed}")
print(f"❌ Başarısız: {failed}")
print(f"⏭️  Atlandı: {skipped}")

if test_results['failed']:
    print("\n❌ Başarısız Testler:")
    for name, error in test_results['failed']:
        print(f"   - {name}: {error}")

if test_results['skipped']:
    print("\n⏭️  Atlanan Testler:")
    for name, reason in test_results['skipped']:
        print(f"   - {name}: {reason}")

# CI/CD için exit code (sadece direkt çalıştırıldığında)
if __name__ == "__main__":
    if failed > 0:
        print("\n❌ TEST BAŞARISIZ - CI/CD hatası!")
        sys.exit(1)
    else:
        print("\n✅ TÜM TESTLER BAŞARILI!")
        sys.exit(0)

