#!/usr/bin/env python3
"""
📊 Hull Information JSON Viewer
hull_information.json dosyasındaki verileri ekranda gösterir
Enter tuşu ile sonraki kayıta geçebilirsiniz
'p' tuşu ile havuz görselleştirmesi görebilirsiniz
"""

import json
import os
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

def clear_screen():
    """Ekranı temizle"""
    os.system('clear' if os.name == 'posix' else 'cls')

def print_separator(char='=', width=80):
    """Ayırıcı çizgi yazdır"""
    print(char * width)

def format_center(text, width=80):
    """Metni ortalı yazdır"""
    return text.center(width)

def print_hull_record(record, index, total):
    """Tek bir hull kayıtını güzel şekilde yazdır"""
    clear_screen()
    
    print_separator('=')
    print(format_center(f"🎯 HULL INFORMATION VIEWER"))
    print_separator('=')
    print(f"📍 Kayıt: {index + 1} / {total}")
    ortam_id = record.get('ortam_id', None)
    if ortam_id:
        print(f"🌍 Ortam: {ortam_id}")
    print_separator('-')
    
    # Hull merkez
    hull_center = record.get('hull_center', [])
    print(f"\n📌 HULL MERKEZİ:")
    print(f"   X: {hull_center[0]:.2f} m")
    print(f"   Y: {hull_center[1]:.2f} m")
    
    # Hull samples
    hull_samples = record.get('hull_samples', [])
    print(f"\n🗺️ HULL SAMPLE'LARI ({len(hull_samples)} nokta):")
    print(f"   İlk 5 nokta:")
    for i, sample in enumerate(hull_samples[:5]):
        print(f"      [{i+1}] X: {sample[0]:.2f}, Y: {sample[1]:.2f}")
    if len(hull_samples) > 5:
        print(f"      ... ve {len(hull_samples) - 5} daha nokta")
    
    # Ada ve Engel noktaları 🔹 YENİ
    ada_engel = record.get('ada_ve_engel_noktalar', [])
    if ada_engel:
        print(f"\n🚧 ADA VE ENGELLERİ ({len(ada_engel)} nokta):")
        print(f"   İlk 5 nokta:")
        for i, point in enumerate(ada_engel[:5]):
            print(f"      [{i+1}] X: {point[0]:.2f}, Y: {point[1]:.2f}")
        if len(ada_engel) > 5:
            print(f"      ... ve {len(ada_engel) - 5} daha nokta")
    else:
        print(f"\n🚧 ADA VE ENGELLERİ: (Veri yok)")

    # 3D Engel bulutu (ham) 🔹 YENİ
    engel_bulutu_3d = record.get('engel_bulutu_3d', [])
    if engel_bulutu_3d:
        print(f"\n☁️ ENGEL BULUTU 3D ({len(engel_bulutu_3d)} nokta):")
        print(f"   İlk 5 nokta:")
        for i, point in enumerate(engel_bulutu_3d[:5]):
            if point is not None and len(point) >= 3:
                print(f"      [{i+1}] X: {point[0]:.2f}, Z: {point[1]:.2f}, Y: {point[2]:.2f}")
            else:
                print(f"      [{i+1}] X: {point[0]:.2f}, Z: {point[1]:.2f}")
        if len(engel_bulutu_3d) > 5:
            print(f"      ... ve {len(engel_bulutu_3d) - 5} daha nokta")
    
    # Formasyon bilgileri
    formasyon_info = record.get('formasyon_information', {})
    if formasyon_info:
        print(f"\n🔲 FORMASYON BİLGİLERİ:")
        print(f"   Formasyon ID: {formasyon_info.get('formasyon_id', 'N/A')}")
        print(f"   Araç Arası Mesafe: {formasyon_info.get('aralik', 0):.1f} m")
        print(f"   Merkez: ({formasyon_info.get('merkez', [0, 0])[0]:.2f}, {formasyon_info.get('merkez', [0, 0])[1]:.2f})")
        print(f"   Yaw: {formasyon_info.get('yaw', 0):.2f}°")
        print(f"   Lider ROV ID: {formasyon_info.get('lider_id', 'N/A')}")
        print(f"   Grup ID: {formasyon_info.get('grup_id', 0)}")
    
    # Grup bilgileri
    grup_info = record.get('grup_bilgisi', {})
    if grup_info and 'rovlar' in grup_info:
        print(f"\n👥 GRUP BİLGİLERİ (Güncel Pozisyonlar):")
        print(f"   ROV Sayısı: {len(grup_info.get('rovlar', []))}")
        for rov in grup_info.get('rovlar', [])[:3]:
            rov_id = rov.get('rov_id', '?')
            pil = rov.get('pil', 0)
            pozisyon = rov.get('pozisyon', [0, 0, 0])
            
            # Pozisyon type'ını kontrol et (tuple, list, dict veya dict yapısı olabilir)
            if isinstance(pozisyon, (list, tuple)) and len(pozisyon) >= 2:
                pos_x, pos_y = pozisyon[0], pozisyon[1]
            elif isinstance(pozisyon, dict):
                pos_x = pozisyon.get('x', 0)
                pos_y = pozisyon.get('y', 0)
            else:
                pos_x, pos_y = 0, 0
            
            print(f"      ROV-{rov_id}: Pil %{pil:.0f}, Pozisyon=({pos_x:.1f}, {pos_y:.1f})")
    
    # Formasyon ROV pozisyonları 🔹 YENİ
    formasyon_rov_pozisyonlari = record.get('formasyon_rov_pozisyonlari', {})
    if formasyon_rov_pozisyonlari:
        print(f"\n🎯 FORMASYON İÇİNDEKİ ROV'LARIN HEDEF POZİSYONLARI:")
        for rov_id, pos in list(formasyon_rov_pozisyonlari.items())[:3]:
            print(f"      ROV-{rov_id}: Hedef=({pos[0]:.1f}, {pos[1]:.1f})")
        if len(formasyon_rov_pozisyonlari) > 3:
            print(f"      ... ve {len(formasyon_rov_pozisyonlari) - 3} daha ROV")
    
    # Timestamp
    timestamp = record.get('timestamp', 'N/A')
    print(f"\n⏰ TIMESTAMP:")
    print(f"   {timestamp}")
    
    print_separator('-')
    print(f"\n💡 İpucu: Enter (sonraki) / 'p' (plot) / 'q' (çık)")
    print()
    return record

def plot_hull_and_rovs(record, index=0, total=1):
    """Hull yapısı ve ROV'ları görselleştir"""
    
    # Navigation state
    nav_state = {'action': None}
    
    def on_key(event):
        """Keyboard event handler"""
        if event.key == 'enter':
            nav_state['action'] = 'next'
            plt.close()
        elif event.key == 'q':
            nav_state['action'] = 'quit'
            plt.close()
    
    # Matplotlib figure oluştur
    fig, ax = plt.subplots(figsize=(12, 10))

    def depth_to_color(y_val, surface_y=0.0, floor_y=-50.0):
        """Derinlige gore gri tonlari (yuzeye yakin koyu, derinde acik).
        -10m uzeri engeller sabit kahverengi.
        """
        if float(y_val) > -10.0:
            return (0.45, 0.3, 0.2)
        depth_span = max(0.001, surface_y - floor_y)
        t = (surface_y - float(y_val)) / depth_span
        t = max(0.0, min(1.0, t))
        dark_gray = (0.2, 0.2, 0.2)
        light_gray = (0.85, 0.85, 0.85)
        return (
            dark_gray[0] + (light_gray[0] - dark_gray[0]) * t,
            dark_gray[1] + (light_gray[1] - dark_gray[1]) * t,
            dark_gray[2] + (light_gray[2] - dark_gray[2]) * t,
        )
    
    # Hull samples
    hull_samples = record.get('hull_samples', [])
    if hull_samples:
        hull_array = np.array(hull_samples)
        # Hull'u kapalı polygon olarak çiz
        polygon = patches.Polygon(
            hull_array, 
            fill=True, 
            alpha=0.3, 
            color='cyan', 
            edgecolor='blue', 
            linewidth=2,
            label='Hull (Güvenli Bölge)'
        )
        ax.add_patch(polygon)
    
    # Hull merkez
    hull_center = record.get('hull_center', [0, 0])
    ax.plot(hull_center[0], hull_center[1], 'g*', markersize=20, label='Hull Merkezi')
    
    # Formasyon merkez
    formasyon_info = record.get('formasyon_information', {})
    formasyon_merkez = formasyon_info.get('merkez', [0, 0])
    ax.plot(formasyon_merkez[0], formasyon_merkez[1], 'r^', markersize=15, label='Formasyon Merkezi')
    
    # ROV'ları göster (Güncel pozisyonlar)
    grup_info = record.get('grup_bilgisi', {})
    if 'rovlar' in grup_info:
        for rov in grup_info['rovlar']:
            rov_id = rov.get('rov_id', 0)
            pozisyon = rov.get('pozisyon', [0, 0, 0])
            
            # Pozisyon type'ını kontrol et
            if isinstance(pozisyon, (list, tuple)) and len(pozisyon) >= 2:
                pos_x, pos_y = pozisyon[0], pozisyon[1]
            elif isinstance(pozisyon, dict):
                pos_x = pozisyon.get('x', 0)
                pos_y = pozisyon.get('y', 0)
            else:
                continue
            
            # ROV noktasını çiz (Güncel - QırmızıDaire)
            ax.plot(pos_x, pos_y, 'ro', markersize=10, label='ROV (Güncel)' if rov == grup_info['rovlar'][0] else '')
            ax.text(pos_x + 3, pos_y + 3, f'R{rov_id}', fontsize=8, fontweight='bold', color='red')
    
    # Formasyon ROV hedef pozisyonlarını göster 🔹 YENİ (Hedef - Mavi)
    formasyon_rov_pozisyonlari = record.get('formasyon_rov_pozisyonlari', {})
    if formasyon_rov_pozisyonlari:
        for rov_id, pos in formasyon_rov_pozisyonlari.items():
            # Hedef pozisyonu mavi kare olarak çiz
            ax.plot(pos[0], pos[1], 'bs', markersize=8, label='Hedef Pozisyon' if rov_id == list(formasyon_rov_pozisyonlari.keys())[0] else '')
            ax.text(pos[0] - 5, pos[1] - 5, f'T{rov_id}', fontsize=7, fontweight='bold', color='blue')
    
    # 3D Engel bulutunu goster (gri tonlar)
    engel_bulutu_3d = record.get('engel_bulutu_3d', [])
    if engel_bulutu_3d:
        xs, ys, colors = [], [], []
        for p in engel_bulutu_3d:
            if p is None or len(p) < 2:
                continue
            xs.append(p[0])
            ys.append(p[1])
            y_val = p[2] if len(p) >= 3 else 0.0
            colors.append(depth_to_color(y_val))
        if xs:
            ax.scatter(
                xs,
                ys,
                c=colors,
                s=20,
                alpha=0.7,
                marker='o',
                linewidth=0.0,
                label=f'Engel Bulutu 3D ({len(xs)} nokta)'
            )

    # Ada ve Engel noktalarını göster (2D)
    ada_engel = record.get('ada_ve_engel_noktalar', [])
    if ada_engel:
        ada_array = np.array(ada_engel)
        ax.scatter(
            ada_array[:, 0],
            ada_array[:, 1],
            c='orange',
            s=50,
            alpha=0.6,
            marker='x',
            linewidth=2,
            label=f'Ada/Engeller ({len(ada_engel)} nokta)'
        )
    
    # Eksen ayarları
    ax.set_xlabel('X (m)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
    ortam_id = record.get('ortam_id', 'Ortam-0')
    ax.set_title(
        f"🎯 Havuz Görselleştirmesi (Kayıt {index + 1}/{total}) - {ortam_id}\n"
        f"Hull Merkezi: ({hull_center[0]:.1f}, {hull_center[1]:.1f})",
        fontsize=14,
        fontweight='bold'
    )
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # Info text
    info_text = "🎮 Kontroller:\n[Enter] Sonraki  |  [q] Çık"
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes, 
            fontsize=10, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    # Keyboard event connect
    fig.canvas.mpl_connect('key_press_event', on_key)
    
    plt.tight_layout()
    plt.show()
    
    return nav_state['action']

def view_hull_information(filename='hull_information.json'):
    """Hull information dosyasını interaktif şekilde göster"""
    
    # Dosya kontrolü
    if not os.path.exists(filename):
        print(f"❌ Hata: '{filename}' dosyası bulunamadı!")
        return
    
    # JSON dosyasını oku
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except Exception as e:
        print(f"❌ Dosya okuma hatası: {e}")
        return
    
    if not data:
        print("⚠️ Dosya boş!")
        return

    # Ortam bazli format destekle
    if isinstance(data, dict):
        records = []
        def _ortam_key_sort(k):
            if isinstance(k, str) and k.startswith('Ortam-'):
                try:
                    return int(k.split('-', 1)[1])
                except (ValueError, IndexError):
                    return 0
            return 0
        for ortam_key in sorted(data.keys(), key=_ortam_key_sort):
            ortam_records = data.get(ortam_key, [])
            if not isinstance(ortam_records, list):
                ortam_records = [ortam_records]
            for rec in ortam_records:
                if isinstance(rec, dict):
                    rec = dict(rec)
                    rec.setdefault('ortam_id', ortam_key)
                records.append(rec)
        data = records
    
    total_records = len(data)
    print(f"✅ {total_records} kayıt bulundu")
    print(f"⏳ İlk kayıt 3 saniye sonra gösterilecek...")
    
    import time
    time.sleep(3)
    
    # Her kayıt için görüntüle
    for index, record in enumerate(data):
        print_hull_record(record, index, total_records)
        
        # Kullanıcı girdisi (loop yapı)
        while True:
            user_input = input("➡️ Enter (sonraki) / 'p' (plot mode) / 'q' (çık): ").strip().lower()
            
            if user_input == 'q':
                print("\n👋 Viewer kapatılıyor...")
                return
            elif user_input == 'p':
                # PLOT MODUNA GİR - Enter/q ile navigate et
                plot_mode_index = index
                while True:
                    try:
                        # Plot aç
                        action = plot_hull_and_rovs(data[plot_mode_index], plot_mode_index, total_records)
                        
                        if action == 'quit':
                            print("\n👋 Plot'tan çıkış yapıldı - Viewer kapatılıyor...")
                            return
                        elif action == 'next':
                            # Sonraki kayda git
                            plot_mode_index += 1
                            if plot_mode_index >= total_records:
                                print("⏹️ Son kayda ulaşıldı")
                                index = total_records - 1
                                break
                            else:
                                continue  # Sonraki plot'u aç
                    except Exception as e:
                        print(f"⚠️ Plot hatası: {e}")
                        break
                
                # Plot modundan çık, ana menüye dön
                if plot_mode_index < total_records:
                    index = plot_mode_index - 1  # İç loop sonrası bir sonraki kayda geçebilmesi için
                break
            elif user_input == '':
                # Enter = sonraki kayda geç
                break
            else:
                print("❓ Geçersiz tuş, tekrar deneyin")
    
    print("\n✅ Viewer kapatıldı!")

if __name__ == '__main__':
    try:
        view_hull_information()
    except KeyboardInterrupt:
        print("\n\n⚠️ Viewer interrupt ile kapatıldı")
        sys.exit(0)
