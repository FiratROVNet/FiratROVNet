from FiratROVNet.simulasyon import Ortam
from FiratROVNet.gnc import Filo,TemelGNC
from GAT.gat_test import FiratAnalizci
from FiratROVNet.config import cfg
from ursina import *
import numpy as np
import os

# ==========================================
# 1. KURULUM VE YAPILANDIRMA
# ==========================================
print("🔵 Fırat-GNC Sistemi Başlatılıyor...")
app = Ortam()
# Simülasyonu oluştur: 6 ROV, 6 Ada, 200m havuz yarıçapı
app.sim_olustur(n_rovs=(4,3,), n_islands=5, havuz_genisligi=200, rov_model='submarine')

# --- Navigasyon ve Kuyruk Değişkenleri ---
nav_queue = []          # Hedefleri tutan liste [{'pos': (x,y,z), 'id': 1}, ...]
current_target_id = None # O anda gidilen hedefin ID'si
target_counter = 0      # Her tıklamada artan benzersiz ID sayacı

# GAT Modeli Yükleme
try: 
    beyin = FiratAnalizci(model_yolu="rov_modeli_multi.pth")
    print("✅ GAT modeli yüklendi.")
except Exception as e: 
    print(f"⚠️ Model yüklenemedi, AI devre dışı: {e}")
    beyin = None

# Filo sistemini otomatik kurulum ile oluştur
filo = Filo()
app.filo = filo
filo.ortam_ref=app

for rov in app.rovs:
    rov.gnc=TemelGNC(rov,filo)

# Minimap otomatik açık (ölçek 1.0)
filo.minimap(scale=1.0)

# Konsol fonksiyonları
app.konsola_ekle("git", lambda rov_id, x, z, y=None, ai=True: filo.git(rov_id, x, z, y, ai))
app.konsola_ekle("move", lambda rov_id, yon, guc=1.0: filo.move(rov_id, yon, guc))
app.konsola_ekle("get", lambda rov_id, veri_tipi: filo.get(rov_id, veri_tipi))
app.konsola_ekle("set", lambda rov_id, ayar_adi, deger: filo.set(rov_id, ayar_adi, deger))
app.konsola_ekle("Ada", lambda ada_id, x=None, y=None: app.Ada(ada_id, x, y))
app.konsola_ekle("ROV", lambda rov_id, x=None, y=None, z=None: app.ROV(rov_id, x, y, z))
app.konsola_ekle("filo", filo)
app.konsola_ekle("rovs", app.rovs)
app.konsola_ekle("cfg", cfg)
app.konsola_ekle("nav_queue", nav_queue) # Kuyruğu konsoldan izleyebilirsin
#app.konsola_ekle("temel_gnc",temel_gnc)

print("✅ Sistem aktif. Minimap üzerinden hedef eklemek için sol tıkla.")

# ==========================================
# 2. ANA DÖNGÜ (UPDATE)
# ==========================================
def update():
    """Ana simülasyon döngüsü."""
    global current_target_id
    
    try:
        # --- 1. NAVİGASYON KUYRUĞU VE VARIŞ YÖNETİMİ ---
        # Lider ROV'un (ID: 0) aktif rotasını al
        aktif_rota = filo._git_nokta_listesi.get(0)
        
        # DURUM A: Hedefe Varıldı mı? (Gidilen bir ID var ama rota bittiyse)
        if current_target_id is not None and not aktif_rota:
            print(f"✅ [NAV] Hedef {current_target_id} noktasına varıldı. Görsel siliniyor.")
            filo.hedef_sil(current_target_id) # 3D ve Minimap görselini temizle
            current_target_id = None # Takip değişkenini sıfırla

        # DURUM B: Yeni Hedefe Başla mı? (ROV boşta ve kuyrukta bekleyen var mı?)
        if not aktif_rota and len(nav_queue) > 0:
            next_data = nav_queue.pop(0) # Kuyruktan ilk hedef paketini al
            target_pos = next_data['pos']
            current_target_id = next_data['id']
            
            print(f"🚀 [NAV] Sıradaki hedefe geçiliyor: ID {current_target_id} | Konum: {target_pos}")
            # A* algoritması ile yolu planla ve gitmeye başla
            filo.git_path(0, target_pos, isaret=True)

        # --- 2. SİSTEM GÜNCELLEMELERİ ---
        app.guncelle_sonar_cizgileri()
        if hasattr(app, 'filo') and app.filo is not None:
            app.filo.execute_queued_commands()

        # --- 3. GAT ANALİZİ ---
        veri = app.simden_veriye()
        ai_aktif = getattr(cfg, 'ai_aktif', True)
        
        if ai_aktif and beyin:
            try: 
                tahminler, _, _ = beyin.analiz_et(veri)
            except Exception as e:
                print(f"⚠️ GAT analiz hatası: {e}")
                tahminler = np.zeros(len(app.rovs), dtype=int)
        else:
            tahminler = np.zeros(len(app.rovs), dtype=int)
        #print(tahminler)
        filo.guncelle_hepsi(tahminler)

        # --- 4. GÖRSELLEŞTİRME (RENKLER VE ETİKETLER) ---
        kod_renkleri = {
            0: color.orange, 1: color.red, 2: color.black, 3: color.yellow, 4: color.magenta
        }
        durum_txts = ["OK", "ENGEL", "CARPISMA", "KOPUK", "UZAK"]
        
        for i, gat_kodu in enumerate(tahminler):
            app.rovs[i].gat_kodu = gat_kodu
            
            # Renk ayarı (Lider sabit kırmızı, diğerleri GAT'a göre)
            if app.rovs[i].role == 1: 
                app.rovs[i].color = color.red
            else: 
                app.rovs[i].color = kod_renkleri.get(gat_kodu, color.white)
            
            # Label (Etiket) ayarları
            app.rovs[i].label.color = app.rovs[i].color
            durum_metni = durum_txts[gat_kodu] if 0 <= gat_kodu < len(durum_txts) else f"GAT:{gat_kodu}"
            
            kuyruk_bilgi = f"\n[Kuyruk: {len(nav_queue)}]" if i == 0 and len(nav_queue) > 0 else ""
            app.rovs[i].label.text = f"{durum_metni}{i}{kuyruk_bilgi}"
        
    except Exception as e:
        print(f"❌ [HATA] Update döngüsü: {e}")

app.set_update_function(update)




# ==========================================
# 3. GİRDİ YÖNETİMİ (MOUSE)
# ==========================================
def input(key):
    global target_counter, nav_queue

    if key == 'p':
        lider_rov = app.rovs[0]
        filo.entity_patlat(lider_rov)
    
    if key == 'left mouse down':
        # Eğer tıklanan nesne minimap ise
        if hasattr(app, 'minimap') and mouse.hovered_entity == app.minimap:
            # 1. Tıklanan yerin yerel koordinatını al (-0.5 ile 0.5 arası)
            local_pos = mouse.point 
            
            # 2. Havuz boyutuna göre (200m yarıçap -> 400m tam çap) koordinata çevir
            havuz_tam_cap = 400 
            sim_x = local_pos.x * havuz_tam_cap
            sim_y = local_pos.y * havuz_tam_cap
            
            # 3. Mevcut derinliği liderden (ID: 0) al
            lider_gps = filo.get(0, "gps")
            mevcut_z = lider_gps[2] if lider_gps else -10
            
            # 4. Benzersiz ID oluştur ve hedefi kaydet
            target_counter += 1
            new_id = target_counter
            new_target_pos = (sim_x, sim_y, mevcut_z)

            # Kuyruğa paket olarak ekle
            nav_queue.append({'pos': new_target_pos, 'id': new_id})
            
            # Görseli hem 3D dünyada hem minimap'te oluştur
            filo._hedef_gorsel_olustur(sim_x, sim_y, mevcut_z, id=new_id, debug=False)
            
            print(f"📥 [KUYRUK] Hedef {new_id} eklendi: ({sim_x:.1f}, {sim_y:.1f}) | Bekleyen: {len(nav_queue)}")

# ==========================================
# 4. ÇALIŞTIRMA
# ==========================================
if __name__ == "__main__":
    # Minimap'e tıklanabilmesi için collider ekle (Box collider)
    if hasattr(app, 'minimap') and app.minimap:
        app.minimap.collider = 'box' 
        
    try: 
        app.run(interaktif=True)
    except KeyboardInterrupt: 
        print("\n🛑 Simülasyon durduruldu.")
    finally: 
        os.system('stty sane')
        os._exit(0)