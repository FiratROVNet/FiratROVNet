# 🔄 `guncelle_hepsi()` Fonksiyonu Açıklaması

## 📋 Fonksiyon Tanımı

```python
def guncelle_hepsi(self, tahminler):
    for i, gnc in enumerate(self.sistemler):
        if i < len(tahminler):
            gnc.guncelle(tahminler[i])
```

## 🎯 Görevi

Bu fonksiyon, GAT (Graph Attention Network) modelinden gelen tehlike tahminlerini alır ve her ROV'un GNC (Guidance, Navigation, Control) sistemine iletir. Her ROV, kendi GAT kodu ile güncellenir ve buna göre hareket eder.

---

## 🔄 Çalışma Akışı

### 1. GAT Analizi → Tahminler
```
Simülasyon Verisi → GAT Modeli → Tahminler Array
```

**Örnek:**
```python
tahminler = [0, 1, 0, 2]
# ROV-0: 0 (OK - Normal)
# ROV-1: 1 (ENGEL - Engel tespit edildi)
# ROV-2: 0 (OK - Normal)
# ROV-3: 2 (CARPISMA - Çarpışma riski)
```

### 2. `guncelle_hepsi()` Çağrısı
```python
filo.guncelle_hepsi(tahminler)
```

### 3. Her ROV İçin İşlem
```python
for i, gnc in enumerate(self.sistemler):
    # i = ROV ID (0, 1, 2, 3...)
    # gnc = ROV'un GNC sistemi (LiderGNC veya TakipciGNC)
    if i < len(tahminler):
        gnc.guncelle(tahminler[i])  # ROV'a GAT kodunu gönder
```

---

## 📊 Detaylı Örnek Senaryo

### Senaryo: 4 ROV'lı Sistem

**Başlangıç Durumu:**
- ROV-0: Lider (LiderGNC)
- ROV-1: Takipçi (TakipciGNC)
- ROV-2: Takipçi (TakipciGNC)
- ROV-3: Takipçi (TakipciGNC)

**GAT Tahminleri:**
```python
tahminler = [0, 1, 0, 2]
```

### Adım Adım İşlem

#### **ROV-0 (Lider) - GAT Kodu: 0 (OK)**
```python
gnc = LiderGNC(...)
gnc.guncelle(0)  # GAT kodu: 0 (OK)
```

**İşlem:**
1. ✅ Manuel kontrol değil → Devam
2. ✅ Hedef var → Devam
3. ✅ AI aktif → Devam
4. Mevcut pozisyon: `(10, 0, 20)`
5. Hedef: `(40, 0, 60)`
6. Fark: `(30, 0, 40)`
7. Yön vektörü: `(0.6, 0, 0.8)` (normalize)
8. GAT kodu 0 → Normal hareket
9. **Sonuç:** ROV-0 hedefe doğru normal hızda ilerler

---

#### **ROV-1 (Takipçi) - GAT Kodu: 1 (ENGEL)**
```python
gnc = TakipciGNC(...)
gnc.guncelle(1)  # GAT kodu: 1 (ENGEL)
```

**İşlem:**
1. ✅ Manuel kontrol değil → Devam
2. ✅ Hedef var → Devam
3. ✅ AI aktif → Devam
4. Mevcut pozisyon: `(35, -10, 50)`
5. Hedef: `(40, -10, 60)`
6. Fark: `(5, 0, 10)`
7. Hedef vektörü: `(0.45, 0, 0.9)` (normalize)
8. **GAT kodu 1 (ENGEL) → Kaçınma:**
   - Kaçınma vektörü: `(0, 1.0, 0) + (hedef_vektörü * -0.5)`
   - Kaçınma vektörü: `(0, 1.0, 0) + (-0.225, 0, -0.45) = (-0.225, 1.0, -0.45)`
   - Nihai vektör: `kaçınma + (hedef * 0.1)`
   - Nihai vektör: `(-0.225, 1.0, -0.45) + (0.045, 0, 0.09) = (-0.18, 1.0, -0.36)`
9. Güç: 0.5 (yavaş hareket)
10. **Sonuç:** ROV-1 yukarı çıkar ve yavaşlar (engel kaçınma)

---

#### **ROV-2 (Takipçi) - GAT Kodu: 0 (OK)**
```python
gnc = TakipciGNC(...)
gnc.guncelle(0)  # GAT kodu: 0 (OK)
```

**İşlem:**
1. ✅ Manuel kontrol değil → Devam
2. ✅ Hedef var → Devam
3. ✅ AI aktif → Devam
4. Mevcut pozisyon: `(40, -10, 50)`
5. Hedef: `(45, -10, 60)`
6. Fark: `(5, 0, 10)`
7. Hedef vektörü: `(0.45, 0, 0.9)` (normalize)
8. **GAT kodu 0 (OK) → Normal hareket:**
   - Nihai vektör: `hedef_vektörü` (kaçınma yok)
9. Güç: 1.0 (normal hız)
10. **Sonuç:** ROV-2 hedefe doğru normal hızda ilerler

---

#### **ROV-3 (Takipçi) - GAT Kodu: 2 (CARPISMA)**
```python
gnc = TakipciGNC(...)
gnc.guncelle(2)  # GAT kodu: 2 (CARPISMA)
```

**İşlem:**
1. ✅ Manuel kontrol değil → Devam
2. ✅ Hedef var → Devam
3. ✅ AI aktif → Devam
4. Mevcut pozisyon: `(45, -10, 55)`
5. Hedef: `(50, -10, 60)`
6. Fark: `(5, 0, 5)`
7. Hedef vektörü: `(0.707, 0, 0.707)` (normalize)
8. **GAT kodu 2 (CARPISMA) → Acil kaçınma:**
   - Kaçınma vektörü: `-hedef_vektörü * 1.5`
   - Kaçınma vektörü: `(-1.06, 0, -1.06)`
   - Nihai vektör: `kaçınma + (hedef * 0.1)`
   - Nihai vektör: `(-1.06, 0, -1.06) + (0.07, 0, 0.07) = (-0.99, 0, -0.99)`
9. Güç: 1.0 (normal hız)
10. **Sonuç:** ROV-3 geri çekilir (çarpışma önleme)

---

## 🎮 Gerçek Zamanlı Kullanım

### main.py'de Kullanım:
```python
def update():
    # 1. Simülasyondan veri al
    veri = app.simden_veriye()
    
    # 2. GAT ile analiz et
    tahminler, _, _ = beyin.analiz_et(veri)
    # tahminler = [0, 1, 0, 2]  # Örnek
    
    # 3. Tüm ROV'ları güncelle
    filo.guncelle_hepsi(tahminler)
    # Her ROV kendi GAT koduna göre hareket eder
```

---

## 🔢 GAT Kodları ve Tepkiler

| GAT Kodu | Anlam | Lider Tepkisi | Takipçi Tepkisi |
|----------|-------|---------------|-----------------|
| **0** | OK (Normal) | Hedefe normal ilerleme | Hedefe normal ilerleme |
| **1** | ENGEL | Sağa sapma | Yukarı çık + yavaşla |
| **2** | CARPISMA | DUR (hız = 0) | Geri çekil |
| **3** | KOPUK | - | Yukarı çık (bağlantı kopması) |
| **5** | UZAK | - | Hızlan (liderden uzak) |

---

## 💡 Önemli Notlar

1. **Her Frame'de Çağrılmalı:**
   - `update()` fonksiyonu her frame'de çalışır
   - Her frame'de yeni GAT tahminleri alınır
   - Her frame'de ROV'lar güncellenir

2. **Sıralı İşlem:**
   - ROV'lar sırayla güncellenir (0, 1, 2, 3...)
   - Her ROV kendi GAT kodunu alır

3. **AI Kontrolü:**
   - AI kapalıysa GAT kodu 0 olarak işlenir
   - ROV'lar tehlike yokmuş gibi davranır

4. **Manuel Kontrol:**
   - Manuel kontrol aktifse güncelleme yapılmaz
   - Kullanıcı manuel kontrol edebilir

---

## 🎯 Özet

`guncelle_hepsi()` fonksiyonu:
- ✅ GAT tahminlerini alır
- ✅ Her ROV'a kendi GAT kodunu iletir
- ✅ ROV'lar GAT koduna göre hareket eder
- ✅ Her frame'de çağrılır (gerçek zamanlı)
- ✅ Lider ve takipçi farklı tepkiler verir

Bu fonksiyon, AI destekli otonom navigasyonun kalbidir! 🚢🤖

