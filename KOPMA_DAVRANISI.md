# 🔌 Kopma (GAT Kodu 3) Davranışı

## 📋 Genel Bakış

GAT kodu 3 (KOPMA) geldiğinde, kopan ROV otomatik olarak sisteme yaklaşır ve iletişim menzili içine girer.

---

## 🎯 Davranış Mantığı

### **Kopma Tespiti:**
- GAT kodu 3 geldiğinde ROV bağlantısı kopmuş demektir
- ROV, diğer ROV'lardan veya liderden uzaklaşmıştır

### **Yaklaşma Stratejisi:**
1. **En Yakın ROV'u Bul:**
   - Öncelik: Lider ROV (eğer varsa)
   - Alternatif: En yakın ROV
   
2. **Yaklaşma Hareketi:**
   - Hedef ROV'a doğru hareket eder
   - Yukarı çıkar (sinyal daha iyi alınır)
   - Hız: %120 (normalden daha hızlı)

3. **İletişim Menzili Kontrolü:**
   - İletişim menzili: `iletisim_menzili` sensör ayarı (varsayılan: 35.0)
   - Menzil içine girince (%80 menzil): Normal hedefe döner
   - Menzil dışındayken: Yaklaşmaya devam eder

---

## 🔄 Kod Akışı

### **TakipciGNC.guncelle() - GAT Kodu 3:**

```python
if gat_kodu == 3:
    # 1. En yakın ROV'u bul (lider öncelikli)
    # 2. Hedef ROV'a yaklaş
    # 3. İletişim menzili içine girince normal hedefe dön
```

### **Yaklaşma Vektörü:**
```python
yaklasma_vektoru = (hedef_rov.position - self.rov.position)
yaklasma_vektoru.y += 5.0  # Yukarı çık (sinyal için)
yaklasma_vektoru = yaklasma_vektoru.normalized()
```

### **Menzil Kontrolü:**
```python
if en_yakin_mesafe < iletisim_menzili * 0.8:  # %80 menzil içindeyse
    # Normal hedefe dön
    nihai_vektor = (self.hedef - self.rov.position).normalized()
else:
    # Hala menzil dışındaysa yaklaşmaya devam et
    nihai_vektor = yaklasma_vektoru
```

---

## 📊 Örnek Senaryo

### **Başlangıç Durumu:**
- ROV-1: Pozisyon `(50, -20, 80)` (uzak)
- ROV-0 (Lider): Pozisyon `(40, 0, 60)`
- İletişim menzili: 35.0 birim
- Mesafe: 45 birim (menzil dışı)

### **GAT Kodu 3 Gelir:**
1. ROV-1 kopma tespit eder
2. En yakın ROV'u bulur: ROV-0 (Lider)
3. ROV-0'a doğru hareket eder
4. Yukarı çıkar (sinyal için)
5. Hız: %120 (normalden daha hızlı)

### **Menzil İçine Girer:**
- Mesafe: 28 birim (< 35.0 * 0.8 = 28)
- Normal hedefe döner
- Formasyona geri döner

---

## ⚙️ Parametreler

### **İletişim Menzili:**
```python
# Sensör ayarlarından
iletisim_menzili = self.rov.sensor_config.get("iletisim_menzili", 35.0)
```

### **Yaklaşma Hızı:**
```python
guc = 1.2  # %120 güç (normalden %20 daha hızlı)
```

### **Yukarı Çıkma:**
```python
yaklasma_vektoru.y += 5.0  # 5 birim yukarı
```

### **Menzil Eşiği:**
```python
if en_yakin_mesafe < iletisim_menzili * 0.8:  # %80 menzil
```

---

## 🎮 Kullanım

### **Otomatik Çalışır:**
Kopma durumu otomatik olarak tespit edilir ve ROV yaklaşır:

```python
# GAT analizi yapılır
tahminler = beyin.analiz_et(veri)
# Eğer ROV-1 için kod 3 gelirse:
# ROV-1 otomatik olarak en yakın ROV'a yaklaşır
```

### **Manuel Kontrol:**
Manuel kontrol açıkken kopma davranışı çalışmaz:

```python
filo.sistemler[1].manuel_kontrol = True
# Kopma davranışı devre dışı
```

---

## 🔧 Özelleştirme

### **İletişim Menzilini Değiştir:**
```python
filo.set(1, 'iletisim_menzili', 50.0)  # 50 birim menzil
```

### **Yaklaşma Hızını Değiştir:**
`gnc.py` dosyasında `guc = 1.2` değerini değiştirebilirsiniz.

---

## 📝 Özet

| Durum | Davranış |
|-------|----------|
| GAT Kodu 3 (KOPMA) | En yakın ROV'a yaklaşır |
| Menzil Dışı | Yaklaşmaya devam eder |
| Menzil İçi (%80) | Normal hedefe döner |
| Manuel Kontrol | Kopma davranışı devre dışı |

**Sonuç:** Kopan ROV'lar otomatik olarak sisteme yaklaşır ve iletişim menzili içinde kalır! 🔌

