#!/usr/bin/env python3
"""
🧪 Test: HullInformationManager.last_hull_center persistence
"""

class SimpleCacheTest:
    def __init__(self):
        self.last_hull_center = None
    
    def test_call(self, hull_center, offset_threshold=25.0):
        """Simulate get_hull_information control logic"""
        print(f"\n📍 Çağrı: hull_center={hull_center}")
        print(f"   Eski last_hull_center={self.last_hull_center}")
        
        # Control
        if self.last_hull_center is not None:
            import math
            distance = math.sqrt(
                (hull_center[0] - self.last_hull_center[0])**2 + 
                (hull_center[1] - self.last_hull_center[1])**2
            )
            print(f"   Mesafe={distance:.6f}m, threshold={offset_threshold}m")
            
            if distance < offset_threshold:
                print(f"   ❌ KONTROL BLOCK - Kaydetme yapılmıyor")
                return None
        else:
            print(f"   ℹ️ İlk veri - kontrol atlanıyor")
        
        # Data processed - update cache
        self.last_hull_center = hull_center
        print(f"   ✅ last_hull_center güncellendi → {self.last_hull_center}")
        return {"hull_center": hull_center, "status": "saved"}

# Test
cache = SimpleCacheTest()

# Simülate 8 çağrıyı
centers = [
    (-9.999998092651367, -122.32689612446651),
    (-9.999998092651367, -122.32689612446651),
    (-9.999998092651367, -122.32689612446651),
    (-9.999998092651367, -122.32689612446651),
    (50.0, -100.0),  # Farklı merkez
    (50.0, -100.0),  # Aynı
    (76.0, -80.0),   # 30m+ uzak
]

print("="*60)
print("🧪 HullInformationManager Kontrol Mekanizması Test")
print("="*60)

for i, center in enumerate(centers, 1):
    result = cache.test_call(center, offset_threshold=25.0)
    status = "✅ SAVED" if result else "❌ BLOCKED"
    print(f"   Sonuç: {status}")

print("\n" + "="*60)
print("✅ Mock test tamamlandı")
print("="*60)
