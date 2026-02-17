from ursina import *
import random

app = Ursina()

# --- 1. AYARLAR ---
DUNYA_GENISLIGI = 100  # Dünyanın boyutu (-50 ile +50 arası)
NOKTA_KALINLIGI = 0.01    # Minimap'teki noktaların piksel büyüklüğü (Kırmızı blok olmaması için burası önemli)

# --- 2. GERÇEK DÜNYADAKİ NESNELER (Simülasyon) ---
# Takip edilecek 100 tane hareketli nesne oluşturuyoruz
hareketli_nesneler = []
for i in range(100):
    e = Entity(
        model='cube', 
        color=color.random_color(), 
        scale=1.5,
        position=(random.uniform(-40, 40), 0.5, random.uniform(-40, 40))
    )
    # Her nesneye rastgele bir hız ve yön veriyoruz
    e.hiz = Vec3(random.uniform(-10, 10), 0, random.uniform(-10, 10))
    hareketli_nesneler.append(e)

# --- 3. MİNİMAP ARKA PLANI (Çerçeve) ---
minimap_bg = Entity(
    parent=camera.ui,       # UI katmanına koyuyoruz
    model='quad',
    color=color.black90,    # Koyu siyah arka plan
    scale=(0.25, 0.25),     # Ekranın %25'i kadar
    origin=(0.5, 0.5),      # Sağ üst köşe hizalaması için
    position=window.top_right
)

# --- 4. TEK ENTITY, BİNLERCE NOKTA (MESH YÖNTEMİ) ---
# Başlangıçta boş bir mesh oluşturuyoruz.
# mode='point' -> Bu mesh'in noktalar çizmesini sağlar.
nokta_modeli = Mesh(vertices=[], mode='point', thickness=NOKTA_KALINLIGI)

nokta_gorseli = Entity(
    parent=minimap_bg,      # Minimap panelinin içine koyuyoruz
    model=nokta_modeli,
    color=color.green,      # Noktaların rengi (İstersen vertex başına renk de verebilirsin)
    z=-0.01                 # Arka planın 1 tık önünde dursun (Z-fighting önler)
)

# --- 5. GÜNCELLEME DÖNGÜSÜ (Her Karede Çalışır) ---
def update():
    yeni_koordinatlar = []
    
    for nesne in hareketli_nesneler:
        # A) Nesneleri gerçek dünyada hareket ettir (Simülasyon kısmı)
        nesne.position += nesne.hiz * time.dt
        
        # Dünyadan çıkmasınlar diye sınırlardan sektiriyoruz
        if abs(nesne.x) > 45: nesne.hiz.x *= -1
        if abs(nesne.z) > 45: nesne.hiz.z *= -1

        # B) KOORDİNAT DÖNÜŞÜMÜ (Critical Step)
        # Dünyadaki (X, Z) koordinatını -> Minimap'teki (X, Y) koordinatına çevir.
        # Dünya -50..50 arasındaysa, Minimap Paneli -0.5..0.5 arasındadır.
        # Bu yüzden pozisyonu dünya genişliğine bölüyoruz.
        
        harita_x = nesne.x / DUNYA_GENISLIGI
        harita_y = nesne.z / DUNYA_GENISLIGI
        
        # Noktaları listeye ekle (Z ekseni minimap için 0'dır)
        yeni_koordinatlar.append(Vec3(harita_x, harita_y, 0))

    # C) MESH GÜNCELLEME (Batching İşlemi)
    # 100 tane Entity'yi güncellemek yerine, tek bir Mesh'in veri listesini değiştiriyoruz.
    nokta_gorseli.model.vertices = yeni_koordinatlar
    
    # generate() komutu, yeni listeyi GPU'ya tek seferde gönderir. Çok hızlıdır.
    nokta_gorseli.model.generate()

# Test için kamera
EditorCamera()

app.run()



