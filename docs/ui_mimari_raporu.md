# FiratROVNet UI — Mimari Analiz Raporu

**Hazırlayan:** Otomatik Analiz (GitHub Copilot)  
**Tarih:** 2026-02  
**Kapsam:** `UI/` paketi — `ana_pencere.py`, `kopru.py`, `baslat.py`, `tema.py`, `paneller/*.py`

---

## 1. Mevcut Mimari

### 1.1 Süreç Modeli

```
┌─────────────────────────┐       ┌─────────────────────────┐
│   Simülasyon Süreci     │       │      UI Süreci           │
│  (main.py / Ursina)     │       │  (PyQt5 / baslat.py)    │
│                         │  IPC  │                          │
│  _ui_durum_yaz()  ──────┼──────▶│ _rov_durumu.json (1 Hz) │
│  (1 Hz JSON yaz)        │       │                          │
│                         │       │                          │
│  _ui_komut_oku()  ◀─────┼───────│ KOMUT_KUYRUĞU.txt       │
│  (2 Hz okuma)           │       │ (kopru.komut_gonder)     │
└─────────────────────────┘       └─────────────────────────┘
```

Aynı-süreç modunda (filo nesnesi doğrudan verildiğinde) dosya IPC devre dışı kalır:

```
main.py → UI.kopru.filo_bagla(filo) → exec(komut) direkt çalışır
```

### 1.2 Katmanlar

| Katman | Dosya(lar) | Sorumluluk |
|--------|-----------|------------|
| **View** | `ana_pencere.py`, `paneller/*.py` | Qt widget'ları, kullanıcı etkileşimi |
| **Bridge** | `kopru.py` | Simülasyon ↔ UI veri akışı, komut gönderimi |
| **Config** | `tema.py` | Renk sabitleri, global stylesheet |
| **Launcher** | `baslat.py` | Qt platform ayarları, başlatma noktası |

### 1.3 IPC Detayı

**Veri akışı (Sim → UI):**
- `_ui_durum_yaz()` → atomic `tmp + rename` → `UI/_rov_durumu.json`
- `kopru.rov_listesi()` → JSON parse → `KomutaMerkezi._periyodik_guncelle()` (700 ms)
- Gecikme toleransı: `_CACHE_SURE = 2.0` saniye

**Komut akışı (UI → Sim):**
- `kopru.komut_gonder()` → arka plan thread → `KOMUT_KUYRUĞU.txt`'ye satır ekleme
- Simülasyon tarafında güvenlik filtresi: `Ortam(`, `Ursina(`, `sim_olustur(` engellenmiş
- `exec()` kullanımı — bkz. §3 Güvenlik

---

## 2. Güçlü Yönler

### 2.1 Mimari Ayrıştırma
- **`kopru.py` köprüsü** iyi bir soyutlama: paneller IPC detayını bilmiyor, sadece `komut_gonder()` çağırıyor.
- **`tema.py` merkezi stil yönetimi**: sabit sayılar panellere yayılmak yerine tek noktadan kontrol ediliyor.
- **Thread-safe sinyal köprüsü**: `SinyalKoprusu.durum_guncellendi` ile arka plan thread'lerinden Qt UI güncellemesi doğru şekilde yapılıyor.

### 2.2 Sağlamlık
- Splitter durumu `QSettings` ile persist ediliyor; kapatma/açmada bozulmuyor.
- `sim_bagli_mi()` zaman-damgası kontrolü ile stale JSON'a karşı koruma var.
- `atomic rename` ile kısmi JSON yazma sorunu engellenmiş.

### 2.3 Test Edilebilirlik
- `QT_QPA_PLATFORM=offscreen` ile CI'da GUI'siz test mümkün.
- Paneller bağımsız instantiate edilebiliyor (sinyal objesi inject ile).

---

## 3. Riskler ve İyileştirme Önerileri

### 3.1 🔴 Kritik: `exec()` Güvenlik Açığı

**Konum:** `UI/kopru.py:79` ve simülasyon tarafı `_ui_komut_oku()`  
**Sorun:** `exec(komut, local_ns)` ile gelen string doğrudan Python yorumlayıcısına veriliyor. Simülasyon tarafındaki kara liste (blocklist) yetersiz savunma — bypass edilebilir.

**Öneri — Komut Enumerasyonu:**
```python
# kopru.py
class Komut(Enum):
    GIT       = "git"
    DUR       = "dur"
    GRUBA_GIT = "gruba_git"
    FORMASYON = "formasyon"

@dataclass
class KomutMesaji:
    tip: Komut
    parametreler: dict[str, float | int | str]

def komut_gonder(mesaj: KomutMesaji, callback=None): ...
```

Bu yaklaşım:
- Keyfi kod çalıştırmayı imkansız kılar
- Serialization/deserialization yapılandırılmış olur (JSON schema)
- `KOMUT_KUYRUĞU.txt` yerine `KOMUT_KUYRUGU.jsonl` (her satır JSON) kullanılabilir

### 3.2 🟡 Orta: Polling Tabanlı Durum Güncellemesi

**Konum:** `KomutaMerkezi._periyodik_guncelle()` — 700ms zamanlayıcı  
**Sorun:** 
- Değişiklik olmadığında da UI yeniden çiziliyor (gereksiz CPU).
- 700ms gecikme, hızlı görevlerde kullanıcıya bayat veri gösterebilir.
- `DurumSekmesi` ayrı bir 2s zamanlayıcı ekliyor → iki ayrı poll döngüsü.

**Öneri — Reaktif Güncelleme:**
```python
# Simülasyon tarafı: değişiklik olduğunda sinyal gönder
class ROVDurum:
    def __setattr__(self, name, value):
        super().__setattr__(name, value)
        if name in ('batarya', 'gat_kodu', 'gorev'):
            self._degisiklik_sinyali.emit(self.id, name, value)
```

Veya daha pragmatik: `rov_listesi()` hash'ini karşılaştırıp değişiklik yoksa Qt widget güncellemesini atla:
```python
def _periyodik_guncelle(self):
    rovlar = rov_listesi()
    h = hash(str(rovlar))
    if h == self._son_hash:
        return
    self._son_hash = h
    # ... güncelle
```

### 3.3 🟡 Orta: Dosya Tabanlı IPC Kısıtları

**Sorun:** `_rov_durumu.json` ve `KOMUT_KUYRUĞU.txt` dosya IPC'si;
- Ölçeklenmiyor (çok istemci desteği yok)
- Hata toleransı kısıtlı (dosya silinirse sessiz fail)
- Komut teslimi garantisiz (Sim 2Hz'den yavaş okursa komutlar sıkışabilir)

**Öneri — WebSocket veya Unix Domain Socket:**
```
Simülasyon: asyncio WebSocket server → ws://localhost:8765
UI:         QWebSocket ile bağlan → mesaj tabanlı iki yönlü iletişim
```
Bu; gecikmesiz anlık güncellemeleri, ack mekanizmasını ve çoklu UI istemcisini destekler. `kopru.py` köprüsü, transport detayını gizleyen soyutlaması korunarak değiştirilebilir.

### 3.4 🟡 Orta: `GorevPanel` Sekmeleri Statik Bağlı

**Sorun:** `AlanTaramaSekmesi`, `AramaKurtarmaSekmesi`, `ImhaSekmesi` her uygulama başlatmasında oluşturuluyor. Görev sekmeleri bağımsız eklenti olarak kayıt edilemez.

**Öneri — Plugin Mimarisi:**
```python
class GorevSekme(ABC):
    @property
    def baslik(self) -> str: ...
    def widget(self) -> QWidget: ...

class GorevPanel(QWidget):
    def sekme_ekle(self, sekme: GorevSekme): ...
```

### 3.5 🟢 Küçük: Tip Güvenliği Eksikliği

**Konum:** `kopru.rov_listesi()` → `list[dict]` döner; dict anahtarları string, tipler belgelenmiş ama enforce edilmemiş.

**Öneri — TypedDict veya dataclass:**
```python
from typing import TypedDict

class ROVDurum(TypedDict):
    id:       int
    rol:      int
    gorev:    str
    gps:      tuple[float, float, float]
    gat_kodu: int
    batarya:  float
    hiz:      float
    grup_id:  int
```

Bu; IDE tamamlamasını iyileştirir, runtime KeyError riskini azaltır.

### 3.6 🟢 Küçük: `SurucuPanel` Magic Number'lar

**Konum:** `ROVKarti` — `setFixedSize(80, 80)`, `setFixedHeight(20)` gibi sabit piksel değerleri.

**Öneri:** `tema.py`'ye `ROV_KART_BOYUTU = 80` gibi sabitler ekle; DPI-aware layoutlar için `sizeHint()` override'ı düşün.

---

## 4. Mimari Karşılaştırma

| Kriter | Mevcut | Önerilen (Uzun Vadeli) |
|--------|--------|------------------------|
| IPC | Dosya tabanlı (poll) | WebSocket (push) |
| Komut Doğrulama | Kara liste + exec() | Enum + schema |
| Durum Dağıtımı | JSON poll | Event-driven sinyal |
| Test Kolaylığı | ✅ offscreen Qt çalışıyor | ✅ Korunur |
| Bağımlılık | PyQt5, dosya sistemi | PyQt5, asyncio/ws |
| Karmaşıklık | Düşük | Orta |

---

## 5. Öncelik Sırası

1. **[Kritik]** `exec()` → komut enumerasyonuna geçiş — güvenlik riski
2. **[Orta]** Polling → hash karşılaştırmalı koşullu güncelleme — düşük maliyetli hızlı kazanım
3. **[Orta]** `ROVDurum` TypedDict — tür güvenliği
4. **[Uzun vadeli]** WebSocket IPC — mimari iyileştirme
5. **[Uzun vadeli]** Görev sekme plugin sistemi — genişletilebilirlik

---

## 6. Değişmeyen Güçlü Yapılar

Aşağıdaki tasarım kararları iyi — dokunmaya gerek yok:

- `kopru.py` köprü soyutlaması (transport bağımsız panel kodu)
- `tema.py` merkezi stil yönetimi  
- `QSplitter` + `QSettings` persist düzeni
- Sinyal köprüsü ile thread-safe UI güncelleme
- Atomic JSON yazma (tmp+rename)
- `_ilk_yerles` / GPS-based başlangıç yerleşimi
- `DurumSekmesi` 2s otomatik HTML tazeleması
