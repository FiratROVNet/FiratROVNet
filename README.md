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

**01-initial-setup.png** - Bu ekran görüntüsü sistemin ilk başlatılması sırasında görülen temel arayüzü göstermektedir. Ekranda simülasyon ortamının başlangıç durumu, ROV'ların (Sualtı Otonom Araçları) başlangıç konumları ve temel kontrol paneli görülebilir. Bu görüntü, kullanıcıya sistemin başarıyla yüklendiğini ve simülasyonun başlamaya hazır olduğunu gösterir. Arayüzde yer alan şekiller ve göstergeler, sistemin farklı bileşenlerinin durumunu ve simülasyon ortamının temel parametrelerini yansıtmaktadır.

![Simülasyon Genel Bakış 1](./Pictures/02-simulation-overview-1.png)
*3D simülasyon ortamının genel görünümü - ROV'ların sualtı konumları ve hareketleri*

**02-simulation-overview-1.png** - Bu görüntü, Ursina Engine tabanlı 3D simülasyon ortamının genel bakışını sunmaktadır. Ekranda sualtı ortamı, ROV'ların (küp veya küresel şekillerle temsil edilen araçlar) konumları, su yüzeyi ve çevresel öğeler görülebilir. ROV'lar farklı renklerle kodlanmış durumları göstermektedir: kırmızı lider araçları, turuncu normal seyir halindeki araçları, sarı kopuk araçları temsil eder. Ekranda görünen çizgiler ve bağlantılar, ROV'lar arasındaki iletişim bağlantılarını ve formasyon yapısını gösterir. Bu görüntü, sistemin fizik motorunun ve görselleştirme bileşenlerinin başarıyla çalıştığını gösterir.

![Simülasyon Genel Bakış 2](./Pictures/03-simulation-overview-2.png)
*Farklı açıdan simülasyon görünümü - Detaylı çevre ve araç konumları*

**03-simulation-overview-2.png** - Bu ekran görüntüsü, simülasyon ortamının farklı bir kamera açısından görünümünü sunmaktadır. Bu açıdan bakıldığında, ROV'ların üç boyutlu konumları, derinlik farklılıkları ve çevresel engeller daha net görülebilir. Ekranda görünen geometrik şekiller (küpler, küreler, düzlemler) sualtı araçlarını, engelleri ve çevresel öğeleri temsil eder. Renk kodlaması sayesinde her ROV'un durumu anlık olarak takip edilebilir. İletişim bağlantıları çizgilerle gösterilmiş, formasyon yapısı ve araçlar arası mesafeler görselleştirilmiştir. Bu görüntü, sistemin çoklu araç koordinasyonunu ve 3D fizik simülasyonunun detaylarını gösterir.

### 💻 Konsol Arayüzü

![Konsol Arayüzü 1](./Pictures/04-console-interface-1.png)
*Canlı konsol arayüzü - Terminal üzerinden Python komutları ile sistem kontrolü*

**04-console-interface-1.png** - Bu görüntü, sistemin "Human-in-the-Loop" özelliğini gösteren canlı konsol arayüzünü göstermektedir. Terminal penceresinde Python interaktif kabuğu (>>>) görülebilir ve kullanıcı simülasyon çalışırken gerçek zamanlı olarak komutlar girebilmektedir. Ekranda görünen komutlar ve çıktılar, ROV'lara görev atama (`git` fonksiyonu), sistem parametrelerini değiştirme (`cfg` nesnesi) ve manuel kontrol işlemlerini göstermektedir. Bu arayüz sayesinde kullanıcı, simülasyonu durdurmadan dinamik olarak sistem davranışını değiştirebilir, görevler atayabilir ve parametreleri ayarlayabilir. Konsol çıktıları, komutların başarıyla uygulandığını ve sistemin yanıt verdiğini gösterir.

![Konsol Arayüzü 2](./Pictures/05-console-interface-2.png)
*Gelişmiş konsol görünümü - Parametre ayarları ve gerçek zamanlı veri takibi*

**05-console-interface-2.png** - Bu ekran görüntüsü, konsol arayüzünün daha gelişmiş kullanım senaryolarını göstermektedir. Terminalde görünen komutlar ve çıktılar, sistem parametrelerinin (`cfg` nesnesi üzerinden) değiştirilmesini, ROV nesnelerine (`rovs` listesi) doğrudan erişimi ve gerçek zamanlı veri takibini içermektedir. Kullanıcı bu arayüz üzerinden modem görünürlüğünü (`goster_modem`), GNC görünürlüğünü (`goster_gnc`) ve AI aktiflik durumunu (`ai_aktif`) kontrol edebilir. Ayrıca, ROV'ların renklerini değiştirme, hareket komutları verme ve parametre ayarlama gibi gelişmiş işlemler gerçekleştirilebilir. Bu görüntü, sistemin esnek ve dinamik kontrol yeteneklerini vurgular.

### ⚓ Formasyon Yönetimi

![Formasyon Görünümü 1](./Pictures/06-formation-view-1.png)
*ROV sürüsünün formasyon görünümü - Çoklu araç koordinasyonu*

**06-formation-view-1.png** - Bu görüntü, ROV sürüsünün belirli bir formasyon yapısında hareket ettiğini göstermektedir. Ekranda görünen geometrik şekiller (küpler veya küreler) ROV'ları temsil ederken, bunları birleştiren çizgiler araçlar arasındaki iletişim bağlantılarını ve formasyon yapısını gösterir. Renk kodlaması sayesinde lider araç (kırmızı), normal seyir halindeki araçlar (turuncu) ve diğer durumlar ayırt edilebilir. Formasyon yapısı, ROV'ların birbirlerine göre konumlarını ve mesafelerini gösterir. Bu görüntü, sistemin çoklu araç koordinasyonu yeteneğini ve Graph Attention Network (GAT) tabanlı dağıtık karar alma mekanizmasının çalışmasını gösterir. Formasyon koruma algoritması, araçların belirli bir düzen içinde hareket etmesini sağlar.

![Formasyon Görünümü 2](./Pictures/09-formation-view-2.png)
*Farklı formasyon tipinin görünümü - Dinamik sürü davranışları*

**09-formation-view-2.png** - Bu ekran görüntüsü, ROV sürüsünün farklı bir formasyon yapısını veya dinamik bir durumunu göstermektedir. Ekranda görünen şekiller ve bağlantılar, araçların yeni bir formasyon düzenine geçişini veya farklı bir görev senaryosunu yansıtabilir. ROV'ların konumları, renkleri ve birbirlerine olan bağlantıları, sistemin dinamik formasyon yönetimi yeteneğini gösterir. Bu görüntü, sistemin farklı formasyon tiplerini desteklediğini ve araçların görev gereksinimlerine göre formasyonlarını değiştirebildiğini gösterir. Formasyon yapısındaki değişiklikler, lider araç veya merkezi koordinasyon noktası etrafında gerçekleşir ve tüm araçlar bu değişikliklere uyum sağlar.

### 🗺️ Harita ve Navigasyon

![Harita Görünümü 1](./Pictures/07-map-view-1.png)
*2D harita görünümü - ROV'ların konumları ve hareket yolları*

**07-map-view-1.png** - Bu görüntü, simülasyon ortamının 2D harita görünümünü göstermektedir. Haritada ROV'lar nokta veya küçük şekillerle temsil edilirken, hareket yolları çizgilerle gösterilir. Harita üzerinde engeller (dikdörtgen veya düzensiz şekiller), hedef noktalar ve güvenlik alanları görülebilir. ROV'ların konumları gerçek zamanlı olarak güncellenir ve her araç için geçmiş hareket yolu (trail) görselleştirilebilir. Bu 2D görünüm, kullanıcıya sistemin genel durumunu ve araçların konumlarını üstten bakış açısıyla sunar. Harita görünümü, navigasyon planlaması ve görev yönetimi için kritik bir araçtır.

![Harita Görünümü 2](./Pictures/08-map-view-2.png)
*Detaylı harita görünümü - Convex hull ve güvenlik alanları*

**08-map-view-2.png** - Bu ekran görüntüsü, harita görünümünün daha detaylı bir versiyonunu göstermektedir. Haritada görünen çokgen şekiller, ROV sürüsünün convex hull (dışbükey örtü) yapısını temsil eder. Bu geometrik şekil, sürünün kapladığı alanı ve araçların dağılımını gösterir. Ekranda görünen daireler veya çokgenler güvenlik alanlarını, engel bölgelerini veya hedef alanlarını temsil edebilir. ROV'ların konumları, hareket yolları ve formasyon yapısı bu görünümde daha net görülebilir. Convex hull hesaplaması, sürü koordinasyonu ve güvenlik analizi için önemli bir metrik sağlar. Bu görüntü, sistemin gelişmiş harita analizi ve görselleştirme yeteneklerini gösterir.

![Yol Bulma (Pathfinding)](./Pictures/11-pathfinding.png)
*A* algoritması ile otomatik yol planlama - Engel kaçınma ve optimal rota hesaplama*

**11-pathfinding.png** - Bu görüntü, sistemin A* (A-star) algoritması kullanarak otomatik yol planlama özelliğini göstermektedir. Harita üzerinde görünen yeşil veya mavi çizgiler, ROV'un başlangıç noktasından hedef noktaya kadar hesaplanan optimal rotayı temsil eder. Kırmızı veya gri şekiller engelleri gösterirken, grid yapısı veya noktalar arama algoritmasının çalışma alanını gösterir. A* algoritması, her bir grid hücresinin maliyetini hesaplayarak en kısa ve güvenli yolu bulur. Bu görüntü, sistemin engel kaçınma yeteneğini ve otomatik navigasyon planlamasını gösterir. ROV'lar, bu algoritma sayesinde karmaşık ortamlarda bile hedeflerine güvenli bir şekilde ulaşabilir.

![Navigasyon Görünümü 1](./Pictures/12-navigation-1.png)
*Navigasyon görünümü - Hedef takibi ve otonom hareket*

**12-navigation-1.png** - Bu ekran görüntüsü, sistemin GNC (Guidance, Navigation, Control) modülünün çalışmasını göstermektedir. Ekranda görünen şekiller ve çizgiler, ROV'un hedef noktasına doğru hareketini ve navigasyon planını gösterir. Hedef nokta genellikle farklı bir renkle (örneğin yeşil veya mavi) işaretlenirken, ROV'un mevcut konumu ve yönü oklarla veya çizgilerle gösterilir. Navigasyon görünümünde, ROV'un hedefe olan mesafesi, yönü ve hızı görselleştirilebilir. Bu görüntü, sistemin otonom navigasyon yeteneğini ve hedef takip algoritmasının çalışmasını gösterir. ROV, hedefe ulaşmak için gereken kontrol komutlarını otomatik olarak hesaplar ve uygular.

![Navigasyon Görünümü 2](./Pictures/13-navigation-2.png)
*Gelişmiş navigasyon - Çoklu hedef yönetimi*

**13-navigation-2.png** - Bu görüntü, sistemin çoklu hedef yönetimi ve gelişmiş navigasyon özelliklerini göstermektedir. Ekranda birden fazla hedef noktası görülebilir ve ROV'lar bu hedeflere sırayla veya paralel olarak hareket edebilir. Harita üzerinde görünen farklı renkli şekiller farklı hedefleri, görevleri veya öncelik seviyelerini temsil edebilir. Navigasyon planı, çoklu hedefleri optimize ederek en verimli rotayı hesaplar. Bu görüntü, sistemin karmaşık görev senaryolarını yönetme yeteneğini ve çoklu hedef optimizasyonunu gösterir. ROV'lar, görev gereksinimlerine göre hedefleri önceliklendirir ve en uygun sırayla ziyaret eder.

![Navigasyon Görünümü 3](./Pictures/14-navigation-3.png)
*Dinamik navigasyon senaryosu - Gerçek zamanlı karar alma*

**14-navigation-3.png** - Bu ekran görüntüsü, sistemin dinamik navigasyon senaryosunu ve gerçek zamanlı karar alma mekanizmasını göstermektedir. Ekranda görünen şekiller ve çizgiler, ROV'un değişen ortam koşullarına göre navigasyon planını güncellediğini gösterir. Yeni engellerin ortaya çıkması, hedef konumlarının değişmesi veya formasyon gereksinimlerinin güncellenmesi durumunda sistem otomatik olarak yeni bir rota hesaplar. Bu görüntü, sistemin adaptif navigasyon yeteneğini ve gerçek zamanlı karar alma mekanizmasını gösterir. ROV, çevresel değişiklikleri algılayarak navigasyon planını dinamik olarak günceller ve en güvenli rotayı seçer. Bu özellik, sistemin gerçek dünya uygulamalarında güvenilir çalışmasını sağlar.

### 🎮 3D Simülasyon ve Final Görünüm

![3D Simülasyon Görünümü](./Pictures/10-3d-simulation-view.png)
*3D simülasyon ortamının detaylı görünümü - Fizik motoru ve görselleştirme*

**10-3d-simulation-view.png** - Bu görüntü, Ursina Engine tabanlı 3D simülasyon ortamının detaylı görünümünü göstermektedir. Ekranda görünen üç boyutlu şekiller (küpler, küreler, düzlemler) ROV'ları, engelleri ve çevresel öğeleri temsil eder. Su yüzeyi, sualtı ortamı ve aydınlatma efektleri fiziksel gerçekçiliği artırır. ROV'ların renkleri durumlarını gösterirken, araçlar arasındaki bağlantı çizgileri iletişim ağını gösterir. Bu görüntü, sistemin fizik motorunun (sürtünme, kaldırma kuvveti, motor itki dinamikleri) ve görselleştirme bileşenlerinin entegre çalışmasını gösterir. 3D simülasyon, kullanıcıya sistemin gerçek dünya davranışını anlamak için zengin bir görsel deneyim sunar.

![Final Genel Bakış](./Pictures/15-final-overview.png)
*Sistemin tam özellikli final görünümü - Tüm bileşenlerin entegre çalışması*

**15-final-overview.png** - Bu ekran görüntüsü, sistemin tüm bileşenlerinin entegre çalıştığı tam özellikli final görünümünü göstermektedir. Ekranda 3D simülasyon ortamı, harita görünümü, formasyon yapısı, navigasyon planları ve konsol çıktıları birlikte görülebilir. ROV'lar farklı renklerle durumlarını gösterirken, iletişim bağlantıları, convex hull yapısı ve hareket yolları görselleştirilmiştir. Bu görüntü, sistemin tüm modüllerinin (GAT yapay zeka, GNC navigasyon, fizik motoru, iletişim simülatörü) birlikte çalıştığını ve karmaşık görev senaryolarını başarıyla yönetebildiğini gösterir. Sistem, çoklu ROV koordinasyonu, otonom navigasyon, engel kaçınma ve formasyon yönetimi gibi tüm özelliklerini entegre bir şekilde sunar. Bu görüntü, Fırat-GNC sisteminin tam kapasitesini ve gerçek dünya uygulamalarına hazır olduğunu gösterir.

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
