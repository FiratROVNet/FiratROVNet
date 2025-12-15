# ❌ List Comprehension'da Atama Hatası

## 🔴 Hata

```python
>>> [i.manuel_kontrol=True for i in filo.sistemler[:]]
  File "<console>", line 1
    [i.manuel_kontrol=True for i in filo.sistemler[:]]
     ^^^^^^^^^^^^^^^^
SyntaxError: cannot assign to attribute here. Maybe you meant '==' instead of '='?
```

## 🔍 Hatanın Nedeni

**List Comprehension'lar expression (ifade) döndürmelidir, statement (deyim) değil.**

- **Expression (İfade):** Bir değer döndüren kod (örn: `x + 1`, `i.manuel_kontrol`)
- **Statement (Deyim):** Bir işlem yapan ama değer döndürmeyen kod (örn: `x = 5`, `i.manuel_kontrol = True`)

List comprehension içinde:
- ✅ **Expression kullanabilirsiniz:** `[i.manuel_kontrol for i in filo.sistemler]` → Değerleri döndürür
- ❌ **Statement kullanamazsınız:** `[i.manuel_kontrol=True for i in filo.sistemler]` → HATA!

---

## ✅ Doğru Kullanım Yöntemleri

### 1. **Döngü Kullanarak (En Basit)**

```python
# Tüm ROV'ları manuel kontrol moduna al
for i in filo.sistemler:
    i.manuel_kontrol = True

# Veya daha kısa:
for gnc in filo.sistemler:
    gnc.manuel_kontrol = True
```

### 2. **List Comprehension ile Değer Döndürme (Atama Yapmadan)**

Eğer sadece değerleri görmek istiyorsanız:

```python
# Manuel kontrol durumlarını görüntüle
>>> [i.manuel_kontrol for i in filo.sistemler]
[False, False, False, False]

# Manuel kontrolü aç
>>> for i in filo.sistemler:
...     i.manuel_kontrol = True

# Tekrar kontrol et
>>> [i.manuel_kontrol for i in filo.sistemler]
[True, True, True, True]
```

### 3. **map() Fonksiyonu ile (Fonksiyonel Yaklaşım)**

```python
# Tüm ROV'ları manuel kontrol moduna al
list(map(lambda gnc: setattr(gnc, 'manuel_kontrol', True), filo.sistemler))

# Veya daha okunabilir:
def manuel_kontrol_ac(gnc):
    gnc.manuel_kontrol = True
    return gnc

list(map(manuel_kontrol_ac, filo.sistemler))
```

### 4. **Tek Satırda Döngü (Pythonic)**

```python
# Tüm ROV'ları manuel kontrol moduna al
[setattr(gnc, 'manuel_kontrol', True) for gnc in filo.sistemler]

# Ancak bu yöntem None listesi döndürür (setattr None döndürür)
# Sadece yan etki için kullanılır, değer döndürmez
```

---

## 🎯 Örnek Kullanımlar

### Senaryo 1: Tüm ROV'ları Manuel Kontrole Al

```python
# Yöntem 1: Döngü (Önerilen)
for gnc in filo.sistemler:
    gnc.manuel_kontrol = True

# Yöntem 2: setattr ile
[setattr(gnc, 'manuel_kontrol', True) for gnc in filo.sistemler]
```

### Senaryo 2: Belirli ROV'ları Manuel Kontrole Al

```python
# İlk 2 ROV'u manuel kontrol moduna al
for gnc in filo.sistemler[:2]:
    gnc.manuel_kontrol = True

# Veya belirli indexler
for i in [0, 2]:
    filo.sistemler[i].manuel_kontrol = True
```

### Senaryo 3: Manuel Kontrol Durumunu Kontrol Et

```python
# Tüm ROV'ların manuel kontrol durumunu görüntüle
>>> [gnc.manuel_kontrol for gnc in filo.sistemler]
[False, False, False, False]

# Manuel kontrol açık olan ROV sayısı
>>> sum([gnc.manuel_kontrol for gnc in filo.sistemler])
0

# Manuel kontrol açık olan ROV'ları bul
>>> [i for i, gnc in enumerate(filo.sistemler) if gnc.manuel_kontrol]
[]
```

### Senaryo 4: Koşullu Atama

```python
# Sadece takipçi ROV'ları manuel kontrol moduna al
for gnc in filo.sistemler:
    if isinstance(gnc, TakipciGNC):
        gnc.manuel_kontrol = True

# Veya list comprehension ile filtreleme
for gnc in [g for g in filo.sistemler if isinstance(g, TakipciGNC)]:
    gnc.manuel_kontrol = True
```

### Senaryo 5: Tüm ROV'ları Otomatik Moda Döndür

```python
# Tüm ROV'ları otomatik kontrol moduna al
for gnc in filo.sistemler:
    gnc.manuel_kontrol = False

# Veya tek satırda
[setattr(gnc, 'manuel_kontrol', False) for gnc in filo.sistemler]
```

---

## 📊 Karşılaştırma

| Yöntem | Okunabilirlik | Hız | Önerilen |
|--------|--------------|-----|----------|
| **Döngü** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ✅ Evet |
| **setattr + list comp** | ⭐⭐ | ⭐⭐⭐⭐⭐ | ⚠️ Sadece yan etki için |
| **map()** | ⭐⭐⭐ | ⭐⭐⭐⭐ | ⚠️ Fonksiyonel programlama |

---

## 💡 Özet

**List Comprehension'da Atama Yapamazsınız:**
```python
❌ [i.manuel_kontrol=True for i in filo.sistemler]  # HATA!
```

**Doğru Kullanım:**
```python
✅ for gnc in filo.sistemler:
      gnc.manuel_kontrol = True

✅ [setattr(gnc, 'manuel_kontrol', True) for gnc in filo.sistemler]  # Yan etki için
```

**Değer Döndürmek İçin:**
```python
✅ [gnc.manuel_kontrol for gnc in filo.sistemler]  # Değerleri döndürür
```

---

## 🔧 Konsol Kullanımı

Konsolda kullanım:

```python
# Tüm ROV'ları manuel kontrol moduna al
>>> for gnc in filo.sistemler:
...     gnc.manuel_kontrol = True

# Kontrol et
>>> [gnc.manuel_kontrol for gnc in filo.sistemler]
[True, True, True, True]

# Otomatik moda döndür
>>> for gnc in filo.sistemler:
...     gnc.manuel_kontrol = False
```

