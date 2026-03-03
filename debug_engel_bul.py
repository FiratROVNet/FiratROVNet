#!/usr/bin/env python3
"""
🔍 Engel_bul() çıktısını debug et - ROV + Engel konumlarını kontrol et
"""

import json
import time
from FiratROVNet.simulasyon import Ortam, ROV
from FiratROVNet.gnc import Filo
from FiratROVNet.config import GATLimitleri

def run_debug_simulation():
    """Simülasyonu başlat, engelleri ve ROV'ları kontrol et"""
    
    print("=" * 80)
    print("🔍 DEBUG: ENGEL_BUL() ANALIZI")
    print("=" * 80)
    
    # Simülasyon ortamı oluştur
    print("\n📍 Ortamı oluşturuluyor...")
    ortam = Ortam(
        width=500, 
        height=500, 
        depth=100,
        n_rovs=6,
        n_islands=15,
        n_obstacles=0,
        debug=False,
        show_window=False  # Headless mode
    )
    
    # Filo oluştur
    print("🚁 Filo oluşturuluyor...")
    filo = Filo(rovs=ortam.rovs, ortam_ref=ortam, debug=False)
    
    # ROV konumlarını al
    print("\n" + "=" * 80)
    print("📌 ROV KONUMLARı (SİMÜLASYON KOORDİNATI):")
    print("=" * 80)
    rov_positions = {}
    for rov in ortam.rovs:
        if hasattr(rov, 'position'):
            pos = rov.position
            rov_positions[rov.id] = {
                'x': pos[0] if hasattr(pos, '__getitem__') else pos.x,
                'z': pos[2] if hasattr(pos, '__getitem__') and len(pos) > 2 else pos.z,
            }
            print(f"ROV-{rov.id}: X={rov_positions[rov.id]['x']:.1f}, Z={rov_positions[rov.id]['z']:.1f}")
    
    # Ada (island) konumlarını al
    print("\n" + "=" * 80)
    print("🏝️ ADA KONUMLARı (SİMÜLASYON KOORDİNATı):")
    print("=" * 80)
    if hasattr(ortam, 'island_positions'):
        for idx, pos in enumerate(ortam.island_positions):
            if pos:
                print(f"Ada-{idx}: X={pos[0]:.1f}, Z={pos[1]:.1f}, Boyut={pos[2] if len(pos) > 2 else 'N/A'}")
    
    # Engel bilgilerini al (raycast test)
    print("\n" + "=" * 80)
    print("🔦 ENGEL_BUL() RAYCAST SONUÇLARI (GATLimitleri.ENGEL = {}m)".format(GATLimitleri.ENGEL))
    print("=" * 80)
    
    for rov_id in range(len(ortam.rovs)):
        engeller = filo.engel_bul(rov_id, menzil=GATLimitleri.ENGEL, debug=False)
        
        if engeller:
            print(f"\n📍 ROV-{rov_id}:")
            distances = []
            for engel in engeller:
                mesafe = engel.get('mesafe', 0)
                yon = engel.get('yon', '?')
                koordinat = engel.get('koordinat', (0, 0))
                distances.append(mesafe)
                print(f"   {yon:>5} yön: {mesafe:.2f}m, koordinat: ({koordinat[0]:.1f}, {koordinat[1]:.1f})")
            
            # İstatistikler
            avg_dist = sum(distances) / len(distances) if distances else 0
            min_dist = min(distances) if distances else 0
            print(f"   ➡️ Ort: {avg_dist:.2f}m, Min: {min_dist:.2f}m")
        else:
            print(f"\n📍 ROV-{rov_id}: Engel YOK (menzil dışında)")
    
    # Engel_bulutu (bulut) analiz
    print("\n" + "=" * 80)
    print("☁️ ENGEL_BULUTU (SENSÖRDENTOPLANAN NOKTALAR):")
    print("=" * 80)
    if hasattr(ortam, 'engel_bulutu') and ortam.engel_bulutu:
        print(f"Toplam nokta: {len(ortam.engel_bulutu)}")
        print(f"İlk 10 nokta:")
        for i, pt in enumerate(ortam.engel_bulutu[:10]):
            if pt is not None and len(pt) >= 4:
                print(f"   [{i}] X={pt[0]:.1f}, Z={pt[1]:.1f}, Y={pt[2]:.1f}, Kaynak={pt[3]}")
            elif pt is not None and len(pt) >= 3:
                print(f"   [{i}] X={pt[0]:.1f}, Z={pt[1]:.1f}, Y={pt[2]:.1f}")
            else:
                print(f"   [{i}] X={pt[0]:.1f}, Z={pt[1]:.1f}")
    else:
        print("Engel_bulutu boş (henüz raycast yapılmamış)")
    
    # ROV'lar ve ada'lar arasındaki mesafeleri hesapla
    print("\n" + "=" * 80)
    print("📏 ROV ↔ ADA MESAFELERI:")
    print("=" * 80)
    
    if hasattr(ortam, 'island_positions'):
        for rov_id, rov_pos in rov_positions.items():
            distances_to_islands = []
            for island_idx, island_pos in enumerate(ortam.island_positions):
                if island_pos:
                    dx = rov_pos['x'] - island_pos[0]
                    dz = rov_pos['z'] - island_pos[1]
                    dist = (dx**2 + dz**2)**0.5
                    distances_to_islands.append((island_idx, dist))
            
            # En yakın 3 ada
            distances_to_islands.sort(key=lambda x: x[1])
            print(f"\nROV-{rov_id} - En yakın adalar:")
            for island_idx, dist in distances_to_islands[:3]:
                print(f"   Ada-{island_idx}: {dist:.1f}m")
    
    print("\n" + "=" * 80)
    print("💡 SONUÇ: Engel_bul() raycast kullanıyor ve genelde 12.5m ise,")
    print("   muhtemelen ROV'lar ada sınırlarının ~12.5m dışında bulunuyor.")
    print("   Ada konumlarını kontrol et (main.py'de nasıl oluşturuluyor?)")
    print("=" * 80)

if __name__ == '__main__':
    try:
        run_debug_simulation()
    except KeyboardInterrupt:
        print("\n⚠️ Kesinti...")
    except Exception as e:
        print(f"\n❌ Hata: {e}")
        import traceback
        traceback.print_exc()
