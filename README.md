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
Fırat Üniversitesi
Otonom Sistemler & Yapay Zeka Laboratuvarı
📜 Lisans

MIT License
