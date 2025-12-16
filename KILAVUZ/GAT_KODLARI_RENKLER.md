# 🎨 GAT Kodları ve Renk Kılavuzu

## 📋 Genel Bakış

GAT (Graph Attention Network) kodları, ROV'ların çevrelerindeki tehlikeleri ve durumları algılamak için kullanılan yapay zeka tahmin kodlarıdır. Her GAT kodu, ROV'un görsel rengini ve davranışını belirler.

---

## 🌈 GAT Kodları ve Renk Tablosu

| GAT Kodu | Renk | Durum | Açıklama | ROV Davranışı |
|----------|------|-------|----------|---------------|
| **0** | 🟠 **Turuncu** | **OK** | Normal, güvenli durum | Hedefe doğru normal hızda ilerleme |
| **1** | 🔴 **Kırmızı** | **ENGEL** | Engel tespit edildi | Engelden uzaklaş, yukarı çık, yavaşla |
| **2** | ⚫ **Siyah** | **CARPISMA** | Çarpışma riski (acil) | Acil kaçınma, en uygun rotayı bul |
| **3** | 🟡 **Sarı** | **KOPUK** | İletişim menzili dışında | Yukarı çık, iletişim kurmaya çalış |
| **5** | 🟣 **Mor** | **UZAK** | Liderden aşırı uzak | Hızlan, lideri yakalamaya çalış |

---

## 📊 Detaylı Açıklamalar

### 🟠 GAT Kodu 0: OK (Normal Durum)

**Renk:** `color.orange` (Turuncu)  
**Durum Metni:** `"OK"`

**Açıklama:**
- ROV güvenli bir durumda
- Herhangi bir engel veya tehlike yok
- Hedefe doğru normal şekilde ilerleyebilir

**ROV Davranışı:**
- ✅ Hedefe doğru normal hızda ilerleme
- ✅ Kaçınma hareketi yok
- ✅ Güç: %100 (normal)

**Kod Örneği:**
```python
# main.py'de renk ataması
kod_renkleri = {0: color.orange, ...}
app.rovs[i].color = kod_renkleri.get(gat_kodu, color.white)
```

---

### 🔴 GAT Kodu 1: ENGEL (Engel Tespit Edildi)

**Renk:** `color.red` (Kırmızı)  
**Durum Metni:** `"ENGEL"`

**Açıklama:**
- ROV'un sensörleri bir engel tespit etti
- Engel, ROV'un yolunda veya yakınında
- Kaçınma hareketi gerekiyor

**ROV Davranışı:**
- ⚠️ Engelden uzaklaşma (kaçınma vektörü)
- ⬆️ Yukarı doğru hareket (+0.3 y bileşeni)
- 🐌 Yavaşlama (güç: %50)
- 🎯 Hedefe doğru yönelme (kaçınma ile birleştirilmiş)

**Kod Örneği:**
```python
# gnc.py - LiderGNC.guncelle()
if gat_kodu == 1:  # ENGEL
    if kacinma_vektoru.length() > 0:
        kacinma_vektoru.y += 0.3  # Biraz yukarı
        kacinma_vektoru = kacinma_vektoru.normalized()
    else:
        kacinma_vektoru = Vec3(1, 0, 0)  # Sağa sap
```

---

### ⚫ GAT Kodu 2: CARPISMA (Çarpışma Riski - Acil)

**Renk:** `color.black` (Siyah)  
**Durum Metni:** `"CARPISMA"`

**Açıklama:**
- ROV başka bir ROV veya engel ile çarpışma riski altında
- **ACİL DURUM** - Hemen kaçınma gerekiyor
- En kritik durum

**ROV Davranışı:**
- 🚨 Acil kaçınma (en uygun rota hesaplanır)
- 🔄 `_en_uygun_rota_bul()` fonksiyonu kullanılır
- 🎯 Hedef yönü göz ardı edilir (sadece kaçınma)
- ⚡ Normal hız (güç: %100)

**Kod Örneği:**
```python
# gnc.py - TemelGNC._yaklasma_onleme_vektoru()
if gat_kodu == 2:  # CARPISMA
    # En uygun rotayı bul (yukarı çıkmak yerine)
    return self._en_uygun_rota_bul(tehlikeli_nesneler, hedef_vektoru, kacinma_mesafesi)

# LiderGNC.guncelle()
if gat_kodu == 2:  # ÇARPISMA: En uygun rota direkt kullan
    yon = kacinma_vektoru if kacinma_vektoru.length() > 0 else Vec3(0, 0, 0)
```

---

### 🟡 GAT Kodu 3: KOPUK (İletişim Kopması)

**Renk:** `color.yellow` (Sarı)  
**Durum Metni:** `"KOPUK"`

**Açıklama:**
- ROV lider veya diğer ROV'larla iletişim menzili dışında
- Sürüden ayrılmış durumda
- İletişimi yeniden kurmaya çalışmalı

**ROV Davranışı:**
- ⬆️ Yukarı doğru hareket (+0.2 y bileşeni)
- 🔍 En yakın ROV'u bulmaya çalış
- 📡 İletişim kurmaya çalış
- 🎯 Hedefe doğru yönelme (kaçınma ile birleştirilmiş)

**Kod Örneği:**
```python
# gnc.py - LiderGNC.guncelle()
elif gat_kodu == 3:  # KOPUK
    if kacinma_vektoru.length() > 0:
        kacinma_vektoru.y += 0.2
        kacinma_vektoru = kacinma_vektoru.normalized()
    else:
        kacinma_vektoru = Vec3(0, 0.2, 0)  # Yukarı
```

---

### 🟣 GAT Kodu 5: UZAK (Liderden Uzak)

**Renk:** `color.magenta` (Mor)  
**Durum Metni:** `"UZAK"`

**Açıklama:**
- Takipçi ROV liderden çok uzakta
- Formasyon bozulmuş durumda
- Lideri yakalamak için hızlanmalı

**ROV Davranışı:**
- ⚡ Hızlanma (güç: %150)
- 🎯 Hedefe doğru normal hareket
- 🚫 Kaçınma yok (normal rota)

**Kod Örneği:**
```python
# gnc.py - TakipciGNC.guncelle()
elif gat_kodu == 5:  # UZAK
    # Normal hareket, kaçınma yok
    pass

# Güç ayarı
if gat_kodu == 5: 
    guc = 1.5  # %150 güç (hızlanma)
```

---

## 🎮 Simülasyonda Görselleştirme

### Renk Ataması (main.py)

```python
# GAT kodlarına göre renk tanımları
kod_renkleri = {
    0: color.orange,   # OK
    1: color.red,      # ENGEL
    2: color.black,    # CARPISMA
    3: color.yellow,   # KOPUK
    5: color.magenta   # UZAK
}

# Durum metinleri
durum_txts = ["OK", "ENGEL", "CARPISMA", "KOPUK", "-", "UZAK"]

# Her ROV için renk ve label güncelleme
for i, gat_kodu in enumerate(tahminler):
    if app.rovs[i].role == 1: 
        # Lider her zaman kırmızı
        app.rovs[i].color = color.red
    else: 
        # Takipçiler GAT koduna göre renklenir
        app.rovs[i].color = kod_renkleri.get(gat_kodu, color.white)
    
    # Label güncelleme
    app.rovs[i].label.text = f"R{i}\n{durum_txts[gat_kodu]}"
```

### Özel Durumlar

**Lider ROV:**
- Her zaman **kırmızı** (`color.red`)
- GAT kodundan bağımsız
- Lider olduğunu görsel olarak belirtir

**Batarya Bitti:**
- ROV rengi **gri** (`color.rgb(100, 100, 100)`)
- GAT kodundan bağımsız
- Hareket etmez

**Sensör Tespiti (GAT olmasa bile):**
- Eğer sensör engel tespit ederse ama GAT kodu 0 ise
- ROV rengi **turuncu-kırmızı** (`color.rgb(255, 100, 0)`)
- GAT'ın tespit edemediği engeller için uyarı

---

## 🔄 GAT Kodlarının Kullanımı

### 1. GAT Analizi

```python
# main.py - update() fonksiyonu
def update():
    # Simülasyondan veri al
    veri = simden_veriye()
    
    # GAT ile analiz et
    tahminler, _, _ = beyin.analiz_et(veri)
    # tahminler = [0, 1, 0, 2]  # Her ROV için GAT kodu
    
    # ROV'ları güncelle
    filo.guncelle_hepsi(tahminler)
```

### 2. ROV Güncelleme

```python
# gnc.py - Filo.guncelle_hepsi()
def guncelle_hepsi(self, tahminler):
    for i, gnc in enumerate(self.sistemler):
        if i < len(tahminler):
            gat_kodu = tahminler[i]
            gnc.guncelle(gat_kodu)  # Her ROV kendi GAT kodunu alır
```

### 3. Davranış Belirleme

```python
# gnc.py - LiderGNC.guncelle() veya TakipciGNC.guncelle()
def guncelle(self, gat_kodu):
    # AI kapalıysa GAT kodunu görmezden gel
    if not self.ai_aktif:
        gat_kodu = 0  # Normal durum
    
    # GAT koduna göre kaçınma vektörü hesapla
    kacinma_vektoru = self._yaklasma_onleme_vektoru(gat_kodu, hedef_vektoru)
    
    # GAT koduna göre özel davranışlar
    if gat_kodu == 1:  # ENGEL
        # Yukarı çık, yavaşla
    elif gat_kodu == 2:  # CARPISMA
        # Acil kaçınma
    # ...
```

---

## 📈 GAT Kodları Öncelik Sırası

GAT kodları öncelik sırasına göre işlenir:

1. **GAT Kodu 2 (CARPISMA)** - En yüksek öncelik
   - Acil durum, hemen kaçınma
   - Hedef göz ardı edilir

2. **GAT Kodu 1 (ENGEL)** - Yüksek öncelik
   - Engelden uzaklaşma
   - Hedefe yönelme devam eder (düşük ağırlıkla)

3. **GAT Kodu 3 (KOPUK)** - Orta öncelik
   - İletişim kurmaya çalış
   - Hedefe yönelme devam eder

4. **GAT Kodu 5 (UZAK)** - Düşük öncelik
   - Hızlan, normal hareket

5. **GAT Kodu 0 (OK)** - Normal durum
   - Herhangi bir özel işlem yok

---

## 💡 Önemli Notlar

1. **AI Kontrolü:**
   - AI kapalıysa (`ai_aktif = False`), tüm GAT kodları 0 olarak işlenir
   - ROV'lar tehlike yokmuş gibi davranır

2. **Manuel Kontrol:**
   - Manuel kontrol aktifse (`manuel_kontrol = True`), GAT kodları işlenmez
   - Kullanıcı tam kontrol sahibidir

3. **Lider ROV:**
   - Lider ROV her zaman kırmızı renkte görünür
   - GAT kodundan bağımsız

4. **Renk Önceliği:**
   - Lider > Batarya Bitti > GAT Kodu > Sensör Tespiti

5. **GAT Kodu 4:**
   - Şu anda kullanılmıyor (boş durum)

---

## 🎯 Özet Tablo

| GAT | Renk | Durum | Öncelik | Güç | Kaçınma | Hedef |
|-----|------|-------|---------|-----|---------|-------|
| 0 | 🟠 Turuncu | OK | Normal | %100 | ❌ | ✅ |
| 1 | 🔴 Kırmızı | ENGEL | Yüksek | %50 | ✅ | ⚠️ |
| 2 | ⚫ Siyah | CARPISMA | **En Yüksek** | %100 | ✅✅ | ❌ |
| 3 | 🟡 Sarı | KOPUK | Orta | %100 | ⚠️ | ✅ |
| 5 | 🟣 Mor | UZAK | Düşük | %150 | ❌ | ✅ |

---

## 📚 İlgili Dosyalar

- **GAT Analizi:** `FiratROVNet/gat.py`
- **GNC Sistemi:** `FiratROVNet/gnc.py`
- **Görselleştirme:** `main.py` (update fonksiyonu)
- **ROV Sınıfı:** `FiratROVNet/simulasyon.py` (ROV class)

---

**Son Güncelleme:** 2025  
**Versiyon:** FiratROVNet-test
