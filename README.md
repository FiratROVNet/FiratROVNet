# 🌊 Fırat-GNC  
### Otonom Sualtı Sürü Sistemi

**Fırat Üniversitesi – Otonom Sistemler & Yapay Zeka Laboratuvarı** bünyesinde geliştirilmiştir.

Fırat-GNC, çoklu **Sualtı Otonom Araçları (ROV/AUV)** ve **Su Üstü Araçları (ASV)** için tasarlanmış,  
**Yapay Zeka Destekli (GAT)**, **Fizik Tabanlı** ve **İletişim Kısıtlı** bir sürü simülasyon ortamıdır.

---

## ✨ Özellikler

### 🤖 Dağıtık Yapay Zeka (GAT)
- Her ROV, **Graph Attention Networks (GAT)** kullanarak komşularından gelen bilgileri işler.
- Engel, çarpışma ve kopma gibi kritik durumları **yerel karar alma** ile tespit eder.

### 📡 Gerçekçi Akustik İletişim
- Sualtı modem simülasyonu
- **Gecikme (Delay)**, **Paket Kaybı (Packet Loss)** ve **Gürültü (Noise)** modelleri

### ⚓ Fizik Motoru
- **Ursina Engine** tabanlı 3D simülasyon
- Sürtünme, kaldırma kuvveti (buoyancy) ve motor itki dinamikleri

### 🎮 Canlı Konsol (Human-in-the-Loop)
- Simülasyon çalışırken **terminal üzerinden anlık Python komutları**
- Görev atama, parametre değiştirme ve manuel müdahale

### 🧠 Otonom Navigasyon (GNC)
- Engel kaçınma
- Hedef takibi
- Sürü formasyonu koruma

---

## 📸 Ekran Görüntüleri

Sistemin farklı özelliklerini ve kullanım senaryolarını gösteren ekran görüntüleri:

### 🚀 İlk Kurulum ve Genel Bakış

![İlk Kurulum](./Pictures/01-initial-setup.png)
*Sistemin ilk başlatılması ve temel arayüz görünümü*

![Simülasyon Genel Bakış 1](./Pictures/02-simulation-overview-1.png)
*3D simülasyon ortamının genel görünümü - ROV'ların sualtı konumları ve hareketleri*

![Simülasyon Genel Bakış 2](./Pictures/03-simulation-overview-2.png)
*Farklı açıdan simülasyon görünümü - Detaylı çevre ve araç konumları*

### 💻 Konsol Arayüzü

![Konsol Arayüzü 1](./Pictures/04-console-interface-1.png)
*Canlı konsol arayüzü - Terminal üzerinden Python komutları ile sistem kontrolü*

![Konsol Arayüzü 2](./Pictures/05-console-interface-2.png)
*Gelişmiş konsol görünümü - Parametre ayarları ve gerçek zamanlı veri takibi*

### ⚓ Formasyon Yönetimi

![Formasyon Görünümü 1](./Pictures/06-formation-view-1.png)
*ROV sürüsünün formasyon görünümü - Çoklu araç koordinasyonu*

![Formasyon Görünümü 2](./Pictures/09-formation-view-2.png)
*Farklı formasyon tipinin görünümü - Dinamik sürü davranışları*

### 🗺️ Harita ve Navigasyon

![Harita Görünümü 1](./Pictures/07-map-view-1.png)
*2D harita görünümü - ROV'ların konumları ve hareket yolları*

![Harita Görünümü 2](./Pictures/08-map-view-2.png)
*Detaylı harita görünümü - Convex hull ve güvenlik alanları*

![Yol Bulma (Pathfinding)](./Pictures/11-pathfinding.png)
*A* algoritması ile otomatik yol planlama - Engel kaçınma ve optimal rota hesaplama*

![Navigasyon Görünümü 1](./Pictures/12-navigation-1.png)
*Navigasyon görünümü - Hedef takibi ve otonom hareket*

![Navigasyon Görünümü 2](./Pictures/13-navigation-2.png)
*Gelişmiş navigasyon - Çoklu hedef yönetimi*

![Navigasyon Görünümü 3](./Pictures/14-navigation-3.png)
*Dinamik navigasyon senaryosu - Gerçek zamanlı karar alma*

### 🎮 3D Simülasyon ve Final Görünüm

![3D Simülasyon Görünümü](./Pictures/10-3d-simulation-view.png)
*3D simülasyon ortamının detaylı görünümü - Fizik motoru ve görselleştirme*

![Final Genel Bakış](./Pictures/15-final-overview.png)
*Sistemin tam özellikli final görünümü - Tüm bileşenlerin entegre çalışması*

---

## 📂 Proje Yapısı

```text
StarProjesi/
│
├── main.py                  # Ana çalıştırıcı (Simülasyonu başlatır)
├── rov_modeli_multi.pth     # Eğitilmiş Yapay Zeka Modeli
│
└── FiratROVNet/             # Çekirdek Kütüphane
    ├── __init__.py
    ├── gat.py               # GAT modeli ve eğitim fonksiyonları
    ├── ortam.py             # Veri seti ve senaryo üretimi
    ├── simulasyon.py        # 3D render & fizik motoru
    ├── iletisim.py          # Akustik modem simülatörü
    ├── gnc.py               # Güdüm, Navigasyon ve Kontrol
    └── config.py            # Canlı ayar yönetimi

🛠️ Kurulum

Gerekli Python kütüphanelerini yükleyin:

pip install torch torch_geometric ursina numpy networkx

🧠 Yapay Zeka Eğitimi

İlk çalıştırmadan önce veya modeli güncellemek için eğitim yapılmalıdır.

    Terminali açın ve Python interaktif moda girin

    Aşağıdaki komutları çalıştırın:

from FiratROVNet import gat, ortam

# 1. Eski modeli sıfırla
gat.reset()

# 2. Eğitimi başlat (Dinamik veri ile)
gat.Train(
    veri_kaynagi=lambda: ortam.veri_uret(n_rovs=None),
    epochs=10000
)

    Eğitim tamamlandığında rov_modeli_multi.pth otomatik olarak oluşturulur.

🚀 Çalıştırma
Linux (Grafik Uyumluluk Modu)

LIBGL_ALWAYS_SOFTWARE=1 python main.py

Windows

python main.py

💻 Canlı Konsol Komutları

Simülasyon başladıktan sonra terminal donmaz.
Arka planda çalışan Python kabuğu (>>>) üzerinden sistemi kontrol edebilirsiniz.
1️⃣ Otonom Görev Atama (git)

git(rov_id, x, z, y, ai=True)

Parametre	Açıklama
x, z	Yatay düzlem koordinatları
y	Derinlik (Negatif = su altı)
ai	True: Zeki Mod / False: Kör Mod

Örnekler:

>>> git(1, 50, 50, -5)
>>> git(2, -20, 100, -10, ai=False)

Toplu Formasyon:

>>> for i in range(4):
...     git(i, i*10, 100, -5)

2️⃣ Sistem Ayarları (cfg)

>>> cfg.goster_modem = True
>>> cfg.goster_gnc = True
>>> cfg.ai_aktif = False

3️⃣ Manuel Müdahale (rovs)

>>> rovs[0].move("ileri", 100)
>>> rovs[1].set("engel_mesafesi", 50.0)

>>> from ursina import color
>>> rovs[2].color = color.green

🌈 Renk Kodları ve Durumlar
Renk	Durum	Açıklama
🔴 Kırmızı	Lider / Engel	Lider araç veya engel algılandı
🟠 Turuncu	Güvenli	Normal seyir
⚫ Siyah	Çarpışma	Acil durum
🟡 Sarı	Kopuk	İletişim menzili dışında
🟣 Mor	Uzak	Liderden aşırı uzak
🛑 Çıkış

Simülasyonu güvenli şekilde kapatmak için:

    ESC veya Q tuşuna basın

👨‍💻 Geliştirici

Ömer Faruk Çelik
Mustafa Polat
Gizem Yılmaz
Fırat Üniversitesi
Otonom Sistemler & Yapay Zeka Laboratuvarı
📜 Lisans

MIT License
