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

**Şekil 1: Sistem Başlangıç Arayüzü**

![Şekil 1: Sistem Başlangıç Arayüzü](./Pictures/01-initial-setup.png)

Şekil 1'de sistemin ilk başlatılması sırasında görülen temel arayüz görüntülenmektedir. Ekranda simülasyon ortamının başlangıç durumu, ROV'ların (Sualtı Otonom Araçları) başlangıç konumları ve temel kontrol paneli görülebilir. Şekil 1, kullanıcıya sistemin başarıyla yüklendiğini ve simülasyonun başlamaya hazır olduğunu gösterir. Arayüzde yer alan şekiller ve göstergeler, sistemin farklı bileşenlerinin durumunu ve simülasyon ortamının temel parametrelerini yansıtmaktadır.

**Şekil 2: 3D Simülasyon Ortamı Genel Görünümü**

![Şekil 2: 3D Simülasyon Ortamı Genel Görünümü](./Pictures/02-simulation-overview-1.png)

Şekil 2'de Ursina Engine tabanlı 3D simülasyon ortamının genel bakışı sunulmaktadır. Şekilde sualtı ortamı, ROV'ların (küp veya küresel şekillerle temsil edilen araçlar) konumları, su yüzeyi ve çevresel öğeler görülebilir. ROV'lar farklı renklerle kodlanmış durumları göstermektedir: kırmızı lider araçları, turuncu normal seyir halindeki araçları, sarı kopuk araçları temsil eder. Şekil 2'de görünen çizgiler ve bağlantılar, ROV'lar arasındaki iletişim bağlantılarını ve formasyon yapısını gösterir. Bu şekil, sistemin fizik motorunun ve görselleştirme bileşenlerinin başarıyla çalıştığını gösterir.

**Şekil 3: Simülasyon Ortamı Alternatif Kamera Açısı**

![Şekil 3: Simülasyon Ortamı Alternatif Kamera Açısı](./Pictures/03-simulation-overview-2.png)

Şekil 3'te simülasyon ortamının farklı bir kamera açısından görünümü sunulmaktadır. Bu açıdan bakıldığında, ROV'ların üç boyutlu konumları, derinlik farklılıkları ve çevresel engeller daha net görülebilir. Şekil 3'te görünen geometrik şekiller (küpler, küreler, düzlemler) sualtı araçlarını, engelleri ve çevresel öğeleri temsil eder. Renk kodlaması sayesinde her ROV'un durumu anlık olarak takip edilebilir. İletişim bağlantıları çizgilerle gösterilmiş, formasyon yapısı ve araçlar arası mesafeler görselleştirilmiştir. Bu şekil, sistemin çoklu araç koordinasyonunu ve 3D fizik simülasyonunun detaylarını gösterir.

### 💻 Konsol Arayüzü

**Şekil 4: Canlı Konsol Arayüzü**

![Şekil 4: Canlı Konsol Arayüzü](./Pictures/04-console-interface-1.png)

Şekil 4'te sistemin "Human-in-the-Loop" özelliğini gösteren canlı konsol arayüzü görüntülenmektedir. Terminal penceresinde Python interaktif kabuğu (>>>) görülebilir ve kullanıcı simülasyon çalışırken gerçek zamanlı olarak komutlar girebilmektedir. Şekil 4'te görünen komutlar ve çıktılar, ROV'lara görev atama (`git` fonksiyonu), sistem parametrelerini değiştirme (`cfg` nesnesi) ve manuel kontrol işlemlerini göstermektedir. Bu arayüz sayesinde kullanıcı, simülasyonu durdurmadan dinamik olarak sistem davranışını değiştirebilir, görevler atayabilir ve parametreleri ayarlayabilir. Konsol çıktıları, komutların başarıyla uygulandığını ve sistemin yanıt verdiğini gösterir.

**Şekil 5: Gelişmiş Konsol Kontrol Paneli**

![Şekil 5: Gelişmiş Konsol Kontrol Paneli](./Pictures/05-console-interface-2.png)

Şekil 5'te konsol arayüzünün daha gelişmiş kullanım senaryoları gösterilmektedir. Terminalde görünen komutlar ve çıktılar, sistem parametrelerinin (`cfg` nesnesi üzerinden) değiştirilmesini, ROV nesnelerine (`rovs` listesi) doğrudan erişimi ve gerçek zamanlı veri takibini içermektedir. Kullanıcı bu arayüz üzerinden modem görünürlüğünü (`goster_modem`), GNC görünürlüğünü (`goster_gnc`) ve AI aktiflik durumunu (`ai_aktif`) kontrol edebilir. Ayrıca, ROV'ların renklerini değiştirme, hareket komutları verme ve parametre ayarlama gibi gelişmiş işlemler gerçekleştirilebilir. Şekil 5, sistemin esnek ve dinamik kontrol yeteneklerini vurgular.

### ⚓ Formasyon Yönetimi

**Şekil 6: ROV Formasyon Yapısı**

![Şekil 6: ROV Formasyon Yapısı](./Pictures/06-formation-view-1.png)

Şekil 6'da ROV sürüsünün belirli bir formasyon yapısında hareket ettiği görülmektedir. Şekilde görünen geometrik şekiller (küpler veya küreler) ROV'ları temsil ederken, bunları birleştiren çizgiler araçlar arasındaki iletişim bağlantılarını ve formasyon yapısını gösterir. Renk kodlaması sayesinde lider araç (kırmızı), normal seyir halindeki araçlar (turuncu) ve diğer durumlar ayırt edilebilir. Formasyon yapısı, ROV'ların birbirlerine göre konumlarını ve mesafelerini gösterir. Şekil 6, sistemin çoklu araç koordinasyonu yeteneğini ve Graph Attention Network (GAT) tabanlı dağıtık karar alma mekanizmasının çalışmasını gösterir. Formasyon koruma algoritması, araçların belirli bir düzen içinde hareket etmesini sağlar.

**Şekil 7: Dinamik Formasyon Geçişi**

![Şekil 7: Dinamik Formasyon Geçişi](./Pictures/09-formation-view-2.png)

Şekil 7'de ROV sürüsünün farklı bir formasyon yapısını veya dinamik bir durumunu gösterilmektedir. Şekilde görünen şekiller ve bağlantılar, araçların yeni bir formasyon düzenine geçişini veya farklı bir görev senaryosunu yansıtabilir. ROV'ların konumları, renkleri ve birbirlerine olan bağlantıları, sistemin dinamik formasyon yönetimi yeteneğini gösterir. Şekil 7, sistemin farklı formasyon tiplerini desteklediğini ve araçların görev gereksinimlerine göre formasyonlarını değiştirebildiğini gösterir. Formasyon yapısındaki değişiklikler, lider araç veya merkezi koordinasyon noktası etrafında gerçekleşir ve tüm araçlar bu değişikliklere uyum sağlar.

### 🗺️ Harita ve Navigasyon

**Şekil 8: 2D Harita Görünümü**

![Şekil 8: 2D Harita Görünümü](./Pictures/07-map-view-1.png)

Şekil 8'de simülasyon ortamının 2D harita görünümü gösterilmektedir. Haritada ROV'lar nokta veya küçük şekillerle temsil edilirken, hareket yolları çizgilerle gösterilir. Harita üzerinde engeller (dikdörtgen veya düzensiz şekiller), hedef noktalar ve güvenlik alanları görülebilir. ROV'ların konumları gerçek zamanlı olarak güncellenir ve her araç için geçmiş hareket yolu (trail) görselleştirilebilir. Şekil 8'deki 2D görünüm, kullanıcıya sistemin genel durumunu ve araçların konumlarını üstten bakış açısıyla sunar. Harita görünümü, navigasyon planlaması ve görev yönetimi için kritik bir araçtır.

**Şekil 9: Detaylı Harita Analizi ve Convex Hull**

![Şekil 9: Detaylı Harita Analizi ve Convex Hull](./Pictures/08-map-view-2.png)

Şekil 9'da harita görünümünün daha detaylı bir versiyonu gösterilmektedir. Haritada görünen çokgen şekiller, ROV sürüsünün convex hull (dışbükey örtü) yapısını temsil eder. Bu geometrik şekil, sürünün kapladığı alanı ve araçların dağılımını gösterir. Şekil 9'da görünen daireler veya çokgenler güvenlik alanlarını, engel bölgelerini veya hedef alanlarını temsil edebilir. ROV'ların konumları, hareket yolları ve formasyon yapısı bu görünümde daha net görülebilir. Convex hull hesaplaması, sürü koordinasyonu ve güvenlik analizi için önemli bir metrik sağlar. Şekil 9, sistemin gelişmiş harita analizi ve görselleştirme yeteneklerini gösterir.

**Şekil 10: A* Algoritması ile Yol Planlama**

![Şekil 10: A* Algoritması ile Yol Planlama](./Pictures/11-pathfinding.png)

Şekil 10'da sistemin A* (A-star) algoritması kullanarak otomatik yol planlama özelliği gösterilmektedir. Harita üzerinde görünen yeşil veya mavi çizgiler, ROV'un başlangıç noktasından hedef noktaya kadar hesaplanan optimal rotayı temsil eder. Kırmızı veya gri şekiller engelleri gösterirken, grid yapısı veya noktalar arama algoritmasının çalışma alanını gösterir. A* algoritması, her bir grid hücresinin maliyetini hesaplayarak en kısa ve güvenli yolu bulur. Şekil 10, sistemin engel kaçınma yeteneğini ve otomatik navigasyon planlamasını gösterir. ROV'lar, bu algoritma sayesinde karmaşık ortamlarda bile hedeflerine güvenli bir şekilde ulaşabilir.

**Şekil 11: GNC Modülü Navigasyon Görünümü**

![Şekil 11: GNC Modülü Navigasyon Görünümü](./Pictures/12-navigation-1.png)

Şekil 11'de sistemin GNC (Guidance, Navigation, Control) modülünün çalışması gösterilmektedir. Şekilde görünen şekiller ve çizgiler, ROV'un hedef noktasına doğru hareketini ve navigasyon planını gösterir. Hedef nokta genellikle farklı bir renkle (örneğin yeşil veya mavi) işaretlenirken, ROV'un mevcut konumu ve yönü oklarla veya çizgilerle gösterilir. Navigasyon görünümünde, ROV'un hedefe olan mesafesi, yönü ve hızı görselleştirilebilir. Şekil 11, sistemin otonom navigasyon yeteneğini ve hedef takip algoritmasının çalışmasını gösterir. ROV, hedefe ulaşmak için gereken kontrol komutlarını otomatik olarak hesaplar ve uygular.

**Şekil 12: Çoklu Hedef Navigasyon Yönetimi**

![Şekil 12: Çoklu Hedef Navigasyon Yönetimi](./Pictures/13-navigation-2.png)

Şekil 12'de sistemin çoklu hedef yönetimi ve gelişmiş navigasyon özellikleri gösterilmektedir. Şekilde birden fazla hedef noktası görülebilir ve ROV'lar bu hedeflere sırayla veya paralel olarak hareket edebilir. Harita üzerinde görünen farklı renkli şekiller farklı hedefleri, görevleri veya öncelik seviyelerini temsil edebilir. Navigasyon planı, çoklu hedefleri optimize ederek en verimli rotayı hesaplar. Şekil 12, sistemin karmaşık görev senaryolarını yönetme yeteneğini ve çoklu hedef optimizasyonunu gösterir. ROV'lar, görev gereksinimlerine göre hedefleri önceliklendirir ve en uygun sırayla ziyaret eder.

**Şekil 13: Dinamik Navigasyon ve Adaptif Rota Planlama**

![Şekil 13: Dinamik Navigasyon ve Adaptif Rota Planlama](./Pictures/14-navigation-3.png)

Şekil 13'te sistemin dinamik navigasyon senaryosu ve gerçek zamanlı karar alma mekanizması gösterilmektedir. Şekilde görünen şekiller ve çizgiler, ROV'un değişen ortam koşullarına göre navigasyon planını güncellediğini gösterir. Yeni engellerin ortaya çıkması, hedef konumlarının değişmesi veya formasyon gereksinimlerinin güncellenmesi durumunda sistem otomatik olarak yeni bir rota hesaplar. Şekil 13, sistemin adaptif navigasyon yeteneğini ve gerçek zamanlı karar alma mekanizmasını gösterir. ROV, çevresel değişiklikleri algılayarak navigasyon planını dinamik olarak günceller ve en güvenli rotayı seçer. Bu özellik, sistemin gerçek dünya uygulamalarında güvenilir çalışmasını sağlar.

### 🎮 3D Simülasyon ve Final Görünüm

**Şekil 14: 3D Fizik Motoru Detaylı Görünümü**

![Şekil 14: 3D Fizik Motoru Detaylı Görünümü](./Pictures/10-3d-simulation-view.png)

Şekil 14'te Ursina Engine tabanlı 3D simülasyon ortamının detaylı görünümü gösterilmektedir. Şekilde görünen üç boyutlu şekiller (küpler, küreler, düzlemler) ROV'ları, engelleri ve çevresel öğeleri temsil eder. Su yüzeyi, sualtı ortamı ve aydınlatma efektleri fiziksel gerçekçiliği artırır. ROV'ların renkleri durumlarını gösterirken, araçlar arasındaki bağlantı çizgileri iletişim ağını gösterir. Şekil 14, sistemin fizik motorunun (sürtünme, kaldırma kuvveti, motor itki dinamikleri) ve görselleştirme bileşenlerinin entegre çalışmasını gösterir. 3D simülasyon, kullanıcıya sistemin gerçek dünya davranışını anlamak için zengin bir görsel deneyim sunar.

**Şekil 15: Entegre Sistem Final Görünümü**

![Şekil 15: Entegre Sistem Final Görünümü](./Pictures/15-final-overview.png)

Şekil 15'te sistemin tüm bileşenlerinin entegre çalıştığı tam özellikli final görünümü gösterilmektedir. Şekilde 3D simülasyon ortamı, harita görünümü, formasyon yapısı, navigasyon planları ve konsol çıktıları birlikte görülebilir. ROV'lar farklı renklerle durumlarını gösterirken, iletişim bağlantıları, convex hull yapısı ve hareket yolları görselleştirilmiştir. Şekil 15, sistemin tüm modüllerinin (GAT yapay zeka, GNC navigasyon, fizik motoru, iletişim simülatörü) birlikte çalıştığını ve karmaşık görev senaryolarını başarıyla yönetebildiğini gösterir. Sistem, çoklu ROV koordinasyonu, otonom navigasyon, engel kaçınma ve formasyon yönetimi gibi tüm özelliklerini entegre bir şekilde sunar. Şekil 15, Fırat-GNC sisteminin tam kapasitesini ve gerçek dünya uygulamalarına hazır olduğunu gösterir.

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
