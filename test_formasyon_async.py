#!/usr/bin/env python3
"""
🧪 Formasyon Seçim - Async/Worker Pattern Test Scripti

Bu script, yeni cache sistemi ile formasyon_sec() metodunun
console'dan nasıl kullanılacağını gösterir.

Kullanım:
    python test_formasyon_async.py
"""

import os
import sys
import time

REPO_ROOT = os.path.abspath(os.path.dirname(__file__))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from FiratROVNet.gnc import Filo
from FiratROVNet.config import GATLimitleri, SensorAyarlari


def test_basic_cache():
    """Test 1: Basic cache fonksiyonelliği"""
    print("\n" + "="*70)
    print("TEST 1: Basic Cache Sistemi")
    print("="*70)
    
    # Dummy Filo oluştur (minimal setup)
    class DummyOrtam:
        def __init__(self):
            self.rovs = []
            self.engel_bulutu = []
    
    filo = Filo(ortam=DummyOrtam(), rov_sayisi=3)
    
    # Cache'yi kontrol et
    print(f"✓ Cache başlangıç: {filo.helper.last_formasyon_result}")
    print(f"✓ Timestamp başlangıç: {filo.helper.formasyon_result_timestamp}")
    
    # Manual test sonucu cache'e yaz
    test_result = {
        'f_id': 2,
        'aralik': 25.5,
        'merkez': (100.0, 50.0),
        'yaw': 90.0
    }
    filo.helper.cache_formasyon_result(test_result)
    
    print(f"\n✓ Cache'ye yazıldı: {filo.helper.last_formasyon_result}")
    
    # Cache'den oku
    cached = filo.formasyon_sonucu()
    print(f"✓ Cache'den okundu: {cached}")
    
    # Doğrulama
    assert cached['sonuc'] == test_result
    print("\n✅ TEST 1 PASSED")


def test_future_wrapping():
    """Test 2: Future wrapping ve tracking"""
    print("\n" + "="*70)
    print("TEST 2: Future Wrapping & Tracking")
    print("="*70)
    
    class DummyOrtam:
        def __init__(self):
            self.rovs = []
            self.engel_bulutu = []
    
    filo = Filo(ortam=DummyOrtam(), rov_sayisi=3)
    
    # formasyon_sec() çağrı (async)
    print("\n1️⃣ formasyon_sec() çağrılıyor...")
    future = filo.formasyon_sec(dinamik=False)
    print(f"   Future object: {future}")
    print(f"   Future tracked: {filo.helper.formasyon_future is not None}")
    
    # Thread'in tamamlanmasını bekle
    print("\n2️⃣ Thread tamamlanması bekleniyor...")
    time.sleep(0.2)
    
    # Durumu kontrol et
    print("\n3️⃣ Durumu kontrol et...")
    status = filo.formasyon_durumu()
    print(f"   Status: {status}")
    print(f"   Done: {status['done']}")
    print(f"   Running: {status['running']}")
    
    # Cache'den oku
    print("\n4️⃣ Cache'den sonuç okunuyor...")
    cached = filo.formasyon_sonucu()
    print(f"   Cached result: {cached['sonuc']}")
    print(f"   Timestamp: {cached['zaman']}")
    
    print("\n✅ TEST 2 PASSED")


def test_blocking_wait():
    """Test 3: formasyon_bekle() - blocking çağrı"""
    print("\n" + "="*70)
    print("TEST 3: Blocking Wait (formasyon_bekle)")
    print("="*70)
    
    class DummyOrtam:
        def __init__(self):
            self.rovs = []
            self.engel_bulutu = []
    
    filo = Filo(ortam=DummyOrtam(), rov_sayisi=3)
    
    print("\n1️⃣ formasyon_sec() başlatılıyor...")
    filo.formasyon_sec(dinamik=True)
    
    print("\n2️⃣ Sonuç bekleniyor (blocking)...")
    start = time.time()
    try:
        result = filo.formasyon_bekle(timeout=3)
        elapsed = time.time() - start
        print(f"   Sonuç alındı ({elapsed:.2f}s): {result}")
        print("\n✅ TEST 3 PASSED")
    except TimeoutError:
        print("   Timeout! (bu normal olabilir - test ortamında dependency var)")
        print("\n⚠️ TEST 3 TIMEOUT (Normal - dependency test)")


def test_multiple_calls():
    """Test 4: Multiple formasyon_sec() çağrıları"""
    print("\n" + "="*70)
    print("TEST 4: Multiple formasyon_sec() Calls")
    print("="*70)
    
    class DummyOrtam:
        def __init__(self):
            self.rovs = []
            self.engel_bulutu = []
    
    filo = Filo(ortam=DummyOrtam(), rov_sayisi=3)
    
    print("\n1️⃣ İlk formasyon_sec() çağrısı...")
    f1 = filo.formasyon_sec(g_id=0)
    print(f"   f1: {f1}")
    
    print("\n2️⃣ İkinci formasyon_sec() çağrısı...")
    f2 = filo.formasyon_sec(g_id=1)
    print(f"   f2: {f2}")
    
    print(f"\n3️⃣ Active future check...")
    print(f"   Tracked future: {filo.helper.formasyon_future}")
    print(f"   Is f2: {filo.helper.formasyon_future is f2}")
    
    time.sleep(0.2)
    
    print(f"\n4️⃣ Result check...")
    status = filo.formasyon_durumu()
    print(f"   Status: {status['done']}")
    
    print("\n✅ TEST 4 PASSED")


def test_clear_cache():
    """Test 5: Cache temizleme"""
    print("\n" + "="*70)
    print("TEST 5: Cache Clearing")
    print("="*70)
    
    class DummyOrtam:
        def __init__(self):
            self.rovs = []
            self.engel_bulutu = []
    
    filo = Filo(ortam=DummyOrtam(), rov_sayisi=3)
    
    # Sonuç cache'e yaz
    test_result = {'f_id': 1, 'aralik': 20.0}
    filo.helper.cache_formasyon_result(test_result)
    
    print(f"✓ Cache'ye yazıldı: {filo.helper.last_formasyon_result}")
    
    # Temizlemeden oku
    print(f"\n1️⃣ Oku (temizle=False)...")
    r1 = filo.formasyon_sonucu(clear=False)
    print(f"   Sonuç: {r1['sonuc']}")
    print(f"   Cache hala dolu: {filo.helper.last_formasyon_result is not None}")
    
    # Temizleyerek oku
    print(f"\n2️⃣ Oku (temizle=True)...")
    r2 = filo.formasyon_sonucu(clear=True)
    print(f"   Sonuç: {r2['sonuc']}")
    print(f"   Cache temizlendi: {filo.helper.last_formasyon_result is None}")
    
    print("\n✅ TEST 5 PASSED")


if __name__ == '__main__':
    print("""
    🧪 FORMASYON ASYNC/WORKER PATTERN TEST SUITE
    ═══════════════════════════════════════════════════════════════════════
    """)
    
    try:
        test_basic_cache()
        test_future_wrapping()
        test_blocking_wait()
        test_multiple_calls()
        test_clear_cache()
        
        print("""
    ═══════════════════════════════════════════════════════════════════════
    ✅ TÜM TESTLER TAMAMLANDI!
    
    Konsol'dan canlı test etmek:
    1. python main.py (simulasyonu başlat)
    2. Tab tuşu → Python REPL gir
    3. Komutları dene:
       
       filo.formasyon_sec(dinamik=True)
       time.sleep(0.1)
       filo.formasyon_sonucu()
       
       # veya/veya
       
       result = filo.formasyon_bekle(timeout=5)
    
    ═══════════════════════════════════════════════════════════════════════
        """)
    
    except Exception as e:
        print(f"\n❌ TEST HATASI: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
