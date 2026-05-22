# FiratROVNet GNC Kılavuzu

Bu kılavuz `FiratROVNet.gnc` içindeki Güdüm, Navigasyon ve Kontrol sisteminin canlı konsolda ve Python kodunda nasıl kullanılacağını anlatır. Eski kullanım kılavuzları korunmuştur; bu dosya GNC komutlarını tek yerde toplamak için yazılmıştır.

## 1. Başlatma

Ana simülasyon:

```bash
python main.py
```

`main.py` içinde GNC sistemi şu sırayla kurulur:

```python
from FiratROVNet.simulasyon import Ortam
from FiratROVNet.gnc import Filo

app = Ortam()
app.sim_olustur(n_rovs=(6, 4), n_islands=4, havuz_genisligi=200, rov_model="submarine")
filo = Filo(ortam_ref=app)
```

`Filo(ortam_ref=app)` çağrısı ROV'lara `gnc`, `sensor`, fizik gövdesi, motorlar, minimap ve varsayılan kamera bağlar.

Canlı konsola otomatik eklenen kısa komutlar:

| Komut | Karşılığı |
|---|---|
| `git(...)` | `filo.git(...)` |
| `move(...)` | `filo.move(...)` |
| `get(...)` | `filo.get(...)` |
| `set(...)` | `filo.set(...)` |
| `filo` | Ana GNC/Filo nesnesi |
| `rovs` | Ortamdaki ROV listesi |
| `nav_queue` | Hedef kuyrukları |

## 2. Koordinat Sistemi

GNC komutları simülasyon koordinatı kullanır:

| Eksen | Anlam |
|---|---|
| `x` | Sağ-sol yatay eksen |
| `y` | İleri-geri yatay eksen |
| `z` | Derinlik. Su yüzeyi `0`, su altı negatiftir |

Ursina/Panda tarafında farklı eksen sırası bulunduğu için dönüşüm gerektiğinde:

```python
from FiratROVNet.gnc import Koordinator

ursina_xyz = Koordinator.sim_to_ursina(sim_x, sim_y, sim_z)
sim_xyz = Koordinator.ursina_to_sim(u_x, u_y, u_z)
```

## 3. ROV Hareket Komutları

### `filo.git(rov_id, x, y, z=None, ai=True, sessiz=True)`

ROV'a otonom hedef verir.

```python
filo.git(0, 50, 60, -20)
filo.git(1, [20, 30, -22])
filo.git(2, [[0, 0], [30, 0], [30, 40]], z=-20)
```

Kullanım notları:

| Kullanım | Açıklama |
|---|---|
| `filo.git(0, 50, 60, -20)` | Tek hedef |
| `filo.git(0, [50, 60, -20])` | Liste ile tek hedef |
| `filo.git(0, [[0, 0], [20, 10]], z=-20)` | Çoklu waypoint |
| `filo.git(0, 3, "rov")` | ROV-0'ı ROV-3'ün güncel konumuna gönderir |

Derinlik verilmezse ROV'un barometre/güncel derinliği korunur. Lider ROV derinliği `RolDerinlikAyarlari` ile sığ aralıkta, takipçiler daha derin aralıkta sınırlandırılır.

### `filo.git_path(rov_id, hedef, ai=True, isaret=False)`

A* yol planlayıcı ile hedefe rota üretir. Statik adaları ve lidar engel bulutunu hesaba katar.

```python
filo.git_path(0, (80, -30, -20), isaret=True)
```

Yol bulunamazsa sistem doğrudan `git()` ile hedefe gider.

### `filo.move(rov_id, yon, guc=1.0, sessiz=True)`

Manuel güç komutudur. `guc` çoğu yönde `0.0-1.0`, `yaw` için `-1.0..1.0` aralığına sıkıştırılır.

```python
filo.move(0, "ileri", 1.0)
filo.move(0, "geri", 0.5)
filo.move(0, "sag", 0.6)
filo.move(0, "sol", 0.6)
filo.move(0, "cik", 0.4)
filo.move(0, "bat", 0.4)
filo.move(0, "yaw", -0.3)
filo.move(0, "dur")
```

Geçerli yönler: `ileri`, `geri`, `sag`, `sol`, `cik`, `bat`, `dur`, `yaw`.

Lider ROV için `bat` manuel hareketi engellenir. Havuz sınırına veya yüzeye/tabana çok yakın hareketlerde sistem güvenlik için komutu reddedebilir.

### `filo.bat_gps(rov_id, z)` ve `filo.bat(rov_id, guc)`

`bat_gps`, ROV'un mevcut yatay konumunu koruyarak yeni derinlik hedefi verir:

```python
filo.bat_gps(0, -22)
```

`bat`, dikey motorları doğrudan çalıştıran düşük seviye test komutudur:

```python
filo.bat(0, 0.3)
```

## 4. Modlar ve Görev Durumu

Her ROV'da `rov.gnc.mod`, `rov.gnc.gorev` ve `rov.gnc.gorev_hedef` alanları vardır.

| Mod | Anlam |
|---|---|
| `0` | Bağımsız / hedefe kendi gider |
| `1` | Takipçi / formasyon-lider takibi |

Grup içindeki lider olmayan ROV'ların modunu değiştirmek:

```python
filo.change_mode(g_id=0, new_mode=0)
filo.change_mode(g_id=0, new_mode=1)
```

Tek ROV modunu okumak:

```python
filo.get(0, "mod")
```

Görev bilgisi:

```python
filo.get(0, "gorev")
filo.get(0, "gorev_hedef")
```

Görevler şu değerleri kullanır: `idle`, `alan_tarama`, `arama_kurtarma`, `imha`.

## 5. Veri Okuma Komutları

### `filo.get(rov_id=None, veri_tipi=None, taraf=None, sessiz=False)`

```python
filo.get(0, "gps")
filo.get(0, "hiz")
filo.get(0, "batarya")
filo.get(0, "rol")
filo.get(0, "yaw")
filo.get(0, "sonar")
filo.get(0, "lidar")
filo.get(0, "lidar", 0)
filo.get(0, "sensor")
filo.get(0, "imu")
filo.get(0, "bar")
filo.get(0, "gps_sinyal")
filo.get()
```

Desteklenen önemli veri tipleri:

| Veri tipi | Dönüş |
|---|---|
| `gps` | Sim koordinatı `(x, y, z)` |
| `hiz` | Hız vektörü |
| `batarya` | `0.0-1.0` batarya |
| `rol` | `1=lider`, `0=takipçi` |
| `yaw` | ROV yaw açısı |
| `sonar` | Öndeki sonar mesafesi, yoksa `-1` |
| `lidar` | `{0,1,2,3}` lidar mesafeleri |
| `engels` | Lidar isabetlerinden hesaplanan engel noktaları |
| `mod` | GNC modu |
| `gorev` | Aktif görev adı |
| `gorev_hedef` | Görev hedefi |
| `sensor` | Sensör nesnesi |
| `imu` | İvme, gyro, manyetik yön ve orientation |
| `bar` | Basınç ve derinlik |
| `sicaklik` | Sıcaklık alanı |
| `gps_sinyal` / `gps_signal` | `1` sinyal var, `0` yok |

Lidar yönleri:

| Lidar | Yön |
|---|---|
| `0` / `l0` | İleri |
| `1` / `l1` | Sağ |
| `2` / `l2` | Sol |
| `3` / `l3` | Dip |

Doğrudan property kullanımı:

```python
rov = filo.find_rov_by_id(0)
print(rov.l0, rov.l1, rov.l2, rov.l3)
```

## 6. Ayar Değiştirme

### `filo.set(rov_id, ayar_adi, deger)`

```python
filo.set(0, "rol", 1)
filo.set(0, "yaw", 90)
filo.set(0, "engel_mesafesi", 25.0)
filo.set(0, "iletisim_menzili", 80.0)
filo.set(0, "min_pil_uyarisi", 0.2)
filo.set(0, "kacinma_mesafesi", 8.0)
```

`rol` lider/takipçi bilgisini ve etiketi değiştirir. Sensör ayarları `rov.sensor_config` içinde tutulur.

## 7. Formasyon Komutları

### `filo.formasyon(formasyon_id="LINE", aralik=None, is_3d=False, lider_koordinat=None, dinamik=True)`

```python
filo.formasyon("LINE", aralik=15)
filo.formasyon("V_SHAPE", aralik=20)
filo.formasyon("CIRCLE", aralik=25, is_3d=True)
```

`lider_koordinat` verilirse sadece pozisyon hesaplar:

```python
pozisyonlar = filo.formasyon("DIAMOND", aralik=20, lider_koordinat=(0, 0, -20))
```

Desteklenen formasyon adları:

`LINE`, `V_SHAPE`, `DIAMOND`, `SQUARE`, `CIRCLE`, `ARROW`, `WEDGE`, `ECHELON`, `COLUMN`, `SPREAD`, `TRIANGLE`, `CROSS`, `STAGGERED`, `WALL`, `STAR`, `PHALANX`, `RECTANGLE`, `HEXAGON`, `WAVE`, `SPIRAL`, `TSHAPE`.

### `filo.formasyon_sec(...)`

Hull ve engel verilerine göre uygun formasyonu otomatik seçer.

```python
sonuc = filo.formasyon_sec(g_id=0, dinamik=True, is_3d=False, offset=50, sessiz=False)
```

Önemli parametreler:

| Parametre | Açıklama |
|---|---|
| `g_id` | Grup ID |
| `dinamik` | Lider hareket ettikçe takipçi hedefleri güncellensin |
| `is_3d` | Derinlik ofsetlerini de kullan |
| `offset` | Hull güvenlik mesafesi |
| `tekrar` | Her kaç çağrıda bir seçim yapılsın |

Sonuç örneği:

```python
{
    "f_id": 0,
    "aralik": 15.0,
    "merkez": (10.0, 20.0),
    "yaw": 90.0,
    "hull_information": {...},
    "formasyon_information": {...}
}
```

Lider bilgisi:

```python
lider_id, lider_gps = filo.find_leader_info(g_id=0)
```

## 8. Hedef ve Navigasyon Kuyruğu

Aktif hedefleri okuma:

```python
filo.hedef()
filo.hedef(rov_id=0)
```

Hedef atama:

```python
filo.hedef((50, 60, -20), rov_id=0, ciz=True)
```

Kalıcı hedef görsellerini silme:

```python
filo.hedef_sil(id=3)
filo.debug_hedefleri_temizle()
```

Minimap sol tık ile seçili ROV'a hedef ekler. Seçili ROV `mod=1` takipçi modundaysa hedef ataması reddedilir. Hedefler `nav_queue` içinde tutulur:

```python
nav_queue
filo.guncelle_navigasyon_kuyrugu()
```

ROV bazlı kuyruk anahtarı `rov_<id>` biçimindedir. Grup bazlı kuyruklarda anahtar grup ID olur.

## 9. Alan Tarama Görevi

### `filo.alan_tarama_gorevi.baslat(grup_id, alan, **kwargs)`

Boustrophedon/lawnmower rota üretir. ROV'ları görev için seçer, gerekirse ayrı görev grubu oluşturur, önce formasyonla alan merkezine yaklaşır, sonra bağımsız taramaya geçer.

Alan biçimleri:

```python
alan = (-80, -60, 80, 60)
alan = (-80, -60, 80, 60, -20)
alan = {"x_min": -80, "y_min": -60, "x_max": 80, "y_max": 60, "z": -20}
alan = {"baslangic": (-80, -60), "genislik": 160, "yukseklik": 120, "z": -20}
```

Başlatma:

```python
plan = filo.alan_tarama_gorevi.baslat(
    grup_id=0,
    alan=(-80, -60, 80, 60, -20),
    derinlik=-20,
    serit_araligi=15,
    sessiz=False,
)
```

Güncelleme ve durdurma:

```python
filo.alan_tarama_gorevi.guncelle()
filo.alan_tarama_gorevi.guncelle(grup_id=0)
filo.alan_tarama_gorevi.durdur(grup_id=0, lideri_takip_et=True)
```

UI sürü panelindeki "Görev Durdur" akışı aynı grup için üç görev tipini birlikte temizler:

```python
filo.alan_tarama_gorevi.durdur(grup_id=0, gorselleri_koru=False)
filo.arama_kurtarma_gorevi.durdur(lideri_takip_et=False, gorselleri_koru=False)
filo.imha_gorevi.durdur(lideri_takip_et=False, gorselleri_koru=False)
ui_minimap_gorev_alan_temizle(app)
```

Yeni bir görev başlatılırken panel önce mevcut alan tarama, arama-kurtarma ve imha görevlerini durdurur; ardından yeni görevi tek komut sırası içinde başlatır. Bu sayede eski görev thread'i yeni planı hemen iptal etmez.

Plan içindeki önemli alanlar:

| Alan | Açıklama |
|---|---|
| `plan.grup_id` | Görev için kullanılan grup |
| `plan.kaynak_grup_id` | İlk çağrıda verilen grup |
| `plan.lider_id` | Tarama lideri |
| `plan.rota_by_rov` | Her ROV için waypoint listesi |
| `plan.asama` | `yaklasma` veya `tarama` |

## 10. Arama Kurtarma

### `filo.arama_kurtarma_baslat(grup_id, alan, **kwargs)`

Alan tarama rotası çalıştırır, görev ROV'larında kamera ve YOLO başlatır. Hedef sınıf bulununca görev tamamlanır.

```python
plan = filo.arama_kurtarma_baslat(
    grup_id=0,
    alan=(-80, -60, 80, 60, -20),
    hedef_siniflari=["person"],
    min_confidence=0.45,
    sessiz=False,
)
```

Güncelleme ve durdurma:

```python
tespit = filo.arama_kurtarma_guncelle()
filo.arama_kurtarma_durdur(lideri_takip_et=True)
```

Tespit dönüşü:

```python
YoloTespit(rov_id=..., class_name=..., confidence=..., bbox=(...))
```

## 11. İmha Görevi

### Koordinat imha

```python
gorevli_rov = filo.koordinat_imha_baslat(
    grup_id=0,
    hedef=(40, 30, -20),
    imha_mesafesi=8,
    sessiz=False,
)
```

### Alan imha

Önce alan içinde YOLO ile hedef arar, tespit yapan ROV'u imha noktasına gönderir.

```python
plan = filo.alan_imha_baslat(
    grup_id=0,
    alan=(-80, -60, 80, 60, -20),
    hedef_siniflari=["boat", "person"],
    derinlik=-20,
    imha_mesafesi=8,
)
```

Güncelleme ve durdurma:

```python
sonuc = filo.imha_guncelle()
filo.imha_durdur(lideri_takip_et=True)
```

Başarılı sonuç:

```python
ImhaSonucu(basarili=True, rov_id=..., hedef=..., mesaj="Imha tamamlandi.")
```

## 12. ROV Değer Önerici

Görev için en uygun ROV'ları batarya, mesafe, görev değiştirme maliyeti ve lider/takipçi durumuna göre puanlar.

```python
hedef = {"gorev_adi": "alan_tarama", "grup_id": 0, "alan": (-80, -60, 80, 60, -20)}
havuz = filo.rov_deger_havuzu(hedef)
rovs = filo.rov_deger_oner(hedef, gereken_rov_sayisi=3)
```

Koordinat hedefi:

```python
filo.rov_deger_oner({"gorev_adi": "imha", "grup_id": 0, "koordinat": (40, 30, -20)}, 1)
```

## 13. Kamera ve YOLO

Kamera:

```python
filo.kamera_ayarla(rov_id=0)
filo.kamera_ayarla(rov_id=1, mesafe=(0, -40, 120), aci=(0, 0, 0), fov=75)
filo.kamera_kaldir(1)
```

YOLO:

```python
filo.yolo_baslat(0)
filo.yolo_baslat(0, model_path="Models-AI/YOLO/yolov8n.pt", islem_hizi=3)
filo.camera_manager.yolo_son_tespitler.get(0)
filo.yolo_durdur(0)
```

YOLO kullanmak için ilgili ROV'da kamera aktif olmalıdır.

## 14. Sensör Paketi

Her ROV'da `rov.sensor` bulunur:

```python
sensor = filo.get(0, "sensor")
sensor.gps_signal
sensor.imu
sensor.bar
sensor.sicaklik
```

IMU yapısı:

```python
{
    "accel": {"x": 0.0, "y": 0.0, "z": 0.0},
    "gyro": {"x": 0.0, "y": 0.0, "z": 0.0},
    "mag": {"x": 0.0, "y": 0.0, "z": 0.0},
    "orientation": {"yaw": 0.0, "pitch": 0.0, "roll": 0.0},
}
```

Barometre:

```python
{"basinc_bar": 1.0, "derinlik": -20.0, "derinlik_m": -20.0}
```

GPS sinyali `z < -5.0` olduğunda `0`, aksi halde `1` döner.

## 15. APF, Engel, Hull ve Vektör Araçları

Engel bulma:

```python
filo.engel_bul(0, menzil=20, debug=False)
filo.get(0, "engels")
filo.get_engel_ve_ada()
```

APF:

```python
filo.apf(0)
filo.apf_guncelle_tum()
filo.apf_temizle()
filo.apf_temizle(0)
```

Vektör çizimi:

```python
filo.vektor((0, 0, -20), (30, 20, -20))
```

Hull:

```python
filo.hull(offset=40)
filo.ada_cevre(offset=0)
filo.get_hull_100_samples(sample_count=100)
info = filo.get_hull_information(sample_count=50, g_id=0, kayit=True)
```

`get_hull_information()` grup bilgisi, lider, formasyon, hull merkezi, örnek noktalar ve ROV hedef pozisyonlarını JSON'a uygun dict olarak döndürür.

## 16. Motor ve PID Komutları

Motor şeması:

```python
filo.motor_sema_kaydet()
```

Motorları doğrudan çalıştırma:

```python
filo.motorlari_calistir(rov_id=0, gucler=[0.1, 0.1, 0.1, 0.1, 0.0, 0.0])
```

Gövde eksen kontrol yardımcıları:

```python
rov = filo.find_rov_by_id(0)
filo.yaw(rov, guc=0.1)
filo.roll(rov, guc=0.1)
filo.pitch(rov, guc=0.1)
filo.roll_koru(rov)
filo.pitch_koru(rov)
```

PID paneli:

```python
filo.toggle_pid_ui()
filo.toggle_pid_ui(True)
filo.set_pid_value("Kp", 0.02)
filo.set_pid_value("Ki", 0.001)
filo.set_pid_value("Kd", 0.005)
```

PID hesaplama:

```python
filo.pid_hesapla(rov, "yaw")
filo.pid_hesapla(rov, "roll")
filo.pid_hesapla(rov, "pitch")
```

APF güç paneli:

```python
filo.apf_guc_panel_rovleri(0, 1, 2)
filo.apf_guc_panel_rovleri(None)
filo.apf_guc_panel_goster()
filo.apf_guc_panel_goster(True)
```

## 17. Hasar, Patlama ve Temizlik

```python
filo.carpisma_enerjisi_hesapla(m1, v1_vec, m2, v2_vec)
filo.rov_hasar_kontrol(rov)
filo.rov_is_hit(0)
filo.entity_patlat(rov, parca_sayisi=80)
filo.rov_verilerini_temizle(0)
```

ROV silindiğinde kamera, sonar çizgileri, hedef/vektör izleri ve grup listesi temizlenir.

## 18. GAT ve Ana Güncelleme Döngüsü

GAT kodları:

| Kod | Anlam |
|---|---|
| `0` | OK |
| `1` | ENGEL |
| `2` | CARPISMA |
| `3` | KOPUK |
| `4` | UZAK olarak renklendirme listesinde kullanılır |

Ana döngüde:

```python
tahminler = [0] * len(app.rovs)
filo.guncelle_gat_analizi(tahminler)
filo.guncelle_hepsi(tahminler, guncelle_gorseller=True, guncelle_lider=True)
```

`guncelle_hepsi()` sırası:

1. Komut kuyruğu ve fizik adımı
2. Navigasyon kuyruğu ve görsel renkler
3. Lider seçimi
4. ROV başına hasar, sensör, GNC ve motor güncellemesi
5. Sonar, minimap ve engel bulutu güncellemesi
6. Alan tarama görevi güncellemesi

## 19. Minimap ve Oyun İçi Kısayollar

`main.py` kısayolları:

| Tuş | İşlev |
|---|---|
| `F` | Makale kalitesinde ekran görüntüsü alır |
| `M` | Motor HUD aç/kapat |
| `B` | PID bar paneli aç/kapat |
| `V` | Rerun kayıt aç/kapat |
| `E` | SAC eğitim HUD aç/kapat |
| `2` / `Num 2` | SAC HUD görünürken sonraki SAC ROV |
| `G` | Sonraki aktif gruba geç |
| `P` | Aktif grubun liderini patlatır |
| `R` | Sonraki ROV kamerasına geç |
| Sol tık | Minimap üstündeyse seçili ROV'a hedef atar |

Minimap komutları:

```python
filo.minimap(True)
filo.minimap(False)
filo.minimap(None)
filo.minimap(True, scale=1.5, grid=20)
filo.minimap("ekle", filo.ada_cevre())
```

## 20. Sık Kullanılan Senaryolar

### Bir ROV'u bağımsız moda alıp hedefe gönderme

```python
rov = filo.find_rov_by_id(2)
rov.gnc.mod = 0
filo.git_path(2, (70, 40, -20), isaret=True)
```

### Takipçileri lider moduna döndürme

```python
filo.change_mode(g_id=0, new_mode=1)
filo.formasyon_sec(g_id=0, dinamik=True)
```

### Alan tarama başlatıp bitişleri izleme

```python
plan = filo.alan_tarama_gorevi.baslat(0, (-100, -80, 100, 80, -20), sessiz=False)
biten = filo.alan_tarama_gorevi.guncelle()
```

### Sensör ve mod kontrolü

```python
for rov in filo.rovs:
    print(
        rov.id,
        filo.get(rov.id, "gps"),
        filo.get(rov.id, "mod"),
        filo.get(rov.id, "gorev"),
        filo.get(rov.id, "lidar"),
    )
```

### YOLO ile arama kurtarma

```python
filo.arama_kurtarma_baslat(
    grup_id=0,
    alan=(-80, -80, 80, 80, -20),
    hedef_siniflari=["person"],
    gereken_rov_sayisi=3,
)

tespit = filo.arama_kurtarma_guncelle()
if tespit:
    print(tespit)
```

### Hull bilgisini kaydetme

```python
info = filo.get_hull_information(sample_count=100, g_id=0, kayit=True)
```

Kayıt varsayılan olarak `hull_information.json` dosyasına append mantığıyla yazılır.

## 21. Hızlı Hata Kontrol Listesi

| Belirti | Kontrol |
|---|---|
| ROV hedefe gitmiyor | `filo.get(id, "mod")`, `filo.hedef(rov_id=id)`, `_git_nokta_listesi` |
| Minimap tıklaması reddediliyor | Seçili ROV `mod=1` olabilir |
| YOLO başlamıyor | Kamera aktif mi, `ultralytics` kurulu mu |
| Alan tarama başlamıyor | Grup içinde `idle` ROV var mı |
| Lider dalmıyor | Lider derinliği güvenlik aralığıyla sınırlandırılır |
| Lidar `-1` dönüyor | O yönde menzil içinde hit yok |
| Formasyon seçilemiyor | Hull/engel verisi yetersiz olabilir; fallback uygulanır |
