#!/usr/bin/env python3
"""
🧪 Test: Hull Distance Control Mekanizması
hull_center'ın eski değerden ne kadar uzakta olduğunu test et
"""

import json
import math

def test_distance_control():
    """Test harness for distance control logic"""
    
    print("\n" + "="*60)
    print("🧪 HULL DISTANCE CONTROL TEST")
    print("="*60)
    
    # Test setup
    offset_threshold = 25.0  # 25 meters
    last_hull_center = None
    test_data = [
        # (test_name, hull_center_input, should_pass)
        ("İLK VERİ", (0.0, 0.0), True),                      # İlk veri - kontrol atlanır
        ("AYNI MERKEZ", (0.0, 0.0), False),                  # Aynı merkez: dist=0m < 25m → BLOCK
        ("MİNİ FARK", (0.001, 0.001), False),                # ~0.0014m fark < 25m → BLOCK
        ("15m FARK", (15.0, 0.0), False),                    # 15m < 25m → BLOCK
        ("30m FARK", (30.0, 0.0), True),                     # Güncelledikten sonra: 30m > 25m → PASS
        ("TEKRAR AYNI", (30.0, 0.0), False),                 # Aynı merkez: dist=0m < 25m → BLOCK
        ("55m ÖTE", (55.0, 0.0), True),                      # Eski (30,0) → Yeni (55,0): 25m = threshold → PASS
    ]
    
    for test_name, hull_center, should_pass in test_data:
        if last_hull_center is None:
            # İlk veri
            print(f"\n✓ {test_name}: İlk veri - kontrol atlanıyor")
            last_hull_center = hull_center
            continue
        
        # Mesafe hesapla
        distance = math.sqrt(
            (hull_center[0] - last_hull_center[0])**2 +
            (hull_center[1] - last_hull_center[1])**2
        )
        
        # Kontrol mantığı: distance < threshold ise BLOCK (False), yoksa PASS (True)
        will_pass = distance >= offset_threshold
        
        # Test sonucu
        status = "✅ PASS" if will_pass == should_pass else "❌ FAIL"
        print(f"\n{status} {test_name}")
        print(f"   Eski merkez: {last_hull_center}")
        print(f"   Yeni merkez: {hull_center}")
        print(f"   Mesafe: {distance:.4f}m < {offset_threshold}m?")
        print(f"   Kaydedilecek mi? {will_pass} (beklenen: {should_pass})")
        
        # Eğer kontrol geçerse, merkezi güncelle
        if will_pass:
            last_hull_center = hull_center
            print(f"   → last_hull_center güncellendi")
    
    print("\n" + "="*60)
    print("✅ Logic testi tamamlandı")
    print("="*60 + "\n")

if __name__ == "__main__":
    test_distance_control()
