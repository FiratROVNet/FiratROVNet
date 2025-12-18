# 🏷️ Git Tag ve Release Yönetimi Rehberi

Bu rehber, FıratROVNet projesinde **versiyon tag'ları** ve **GitHub release'leri** nasıl oluşturulacağını açıklar.

---

## 📋 İçindekiler

1. [Versiyon Numaralandırma](#versiyon-numaralandırma)
2. [Git Tag Oluşturma](#git-tag-oluşturma)
3. [GitHub Release Oluşturma](#github-release-oluşturma)
4. [Otomatik Release Süreci](#otomatik-release-süreci)
5. [Örnek Senaryolar](#örnek-senaryolar)

---

## 🔢 Versiyon Numaralandırma

### Semantic Versioning (SemVer)

FıratROVNet projesi **Semantic Versioning** kullanır: `MAJOR.MINOR.PATCH`

- **MAJOR** (1.x.x): Geriye dönük uyumsuz API değişiklikleri
- **MINOR** (x.1.x): Yeni özellikler (geriye dönük uyumlu)
- **PATCH** (x.x.1): Hata düzeltmeleri (geriye dönük uyumlu)

### Örnekler

```
v1.7.2 → v1.7.3  # PATCH: Bug fix
v1.7.2 → v1.8.0  # MINOR: Yeni özellik eklendi
v1.7.2 → v2.0.0  # MAJOR: Büyük değişiklik, API değişti
```

---

## 🏷️ Git Tag Oluşturma

### 1. Versiyon Numarasını Güncelle

Önce `FiratROVNet/__init__.py` dosyasındaki versiyonu güncelle:

```python
__version__ = "1.7.3"  # Yeni versiyon
```

### 2. Değişiklikleri Commit Et

```bash
git add FiratROVNet/__init__.py
git commit -m "chore: bump version to 1.7.3"
git push origin main
```

### 3. Tag Oluştur

#### Lightweight Tag (Basit)

```bash
git tag v1.7.3
git push origin v1.7.3
```

#### Annotated Tag (Önerilen - Mesajlı)

```bash
git tag -a v1.7.3 -m "Release v1.7.3

- Sonar sistemi güncellendi (-1: kopuk, 0: OK, 1: engel)
- GAT kod 1 davranışı eski haline getirildi
- Batarya sistemi 0-1 arası normalize edildi"
git push origin v1.7.3
```

### 4. Tag'ları Kontrol Et

```bash
# Tüm tag'ları listele
git tag -l

# Tag detaylarını görüntüle
git show v1.7.3

# Tag'ları versiyon sırasına göre listele
git tag -l | sort -V
```

### 5. Tag Silme (Gerekirse)

```bash
# Local tag'ı sil
git tag -d v1.7.3

# Remote tag'ı sil
git push origin --delete v1.7.3
```

---

## 🚀 GitHub Release Oluşturma

### Yöntem 1: GitHub Web Arayüzü (Kolay)

1. **GitHub Repository'ye Git**
   - `https://github.com/FiratROVNet/FiratROVNet/releases`

2. **"Draft a new release"** veya **"Create a new release"** butonuna tıkla

3. **Release Bilgilerini Doldur**
   - **Tag version**: `v1.7.3` (dropdown'dan seç veya yeni oluştur)
   - **Release title**: `v1.7.3 - Sonar Sistemi Güncellemesi`
   - **Description**: Değişiklikleri listele

4. **Release Notes Örneği**:

```markdown
## 🎉 Release v1.7.3

### ✨ Yeni Özellikler
- Sonar sistemi güncellendi: -1 (kopuk), 0 (OK), 1 (engel)

### 🐛 Hata Düzeltmeleri
- GAT kod 1 davranışı eski haline getirildi
- Batarya sistemi 0-1 arası normalize edildi

### 📝 Değişiklikler
- `simden_veriye()` fonksiyonu `Ortam` sınıfına taşındı
- `main.py` sadeleştirildi

### 📦 İndirme
Bu release'i kullanmak için:
```bash
git checkout v1.7.3
```

veya

```bash
pip install git+https://github.com/FiratROVNet/FiratROVNet.git@v1.7.3
```
```

5. **"Publish release"** butonuna tıkla

### Yöntem 2: GitHub CLI (gh)

```bash
# GitHub CLI ile release oluştur
gh release create v1.7.3 \
  --title "v1.7.3 - Sonar Sistemi Güncellemesi" \
  --notes "## 🎉 Release v1.7.3

### ✨ Yeni Özellikler
- Sonar sistemi güncellendi

### 🐛 Hata Düzeltmeleri
- GAT kod 1 davranışı eski haline getirildi"
```

---

## 🤖 Otomatik Release Süreci

### Önerilen Workflow

1. **Geliştirme** → `main` branch'inde çalış
2. **Test** → Değişiklikleri test et
3. **Versiyon Güncelle** → `__init__.py`'de versiyonu artır
4. **Commit & Push** → Versiyon commit'ini at
5. **Tag Oluştur** → Annotated tag oluştur ve push et
6. **GitHub Release** → Web arayüzünden release oluştur

### Hızlı Komut Seti

```bash
# 1. Versiyonu güncelle (manuel olarak __init__.py'yi düzenle)
# 2. Commit ve push
git add FiratROVNet/__init__.py
git commit -m "chore: bump version to 1.7.3"
git push origin main

# 3. Tag oluştur ve push et
git tag -a v1.7.3 -m "Release v1.7.3: Sonar sistemi güncellemesi"
git push origin v1.7.3

# 4. GitHub release oluştur (web arayüzünden veya gh CLI ile)
gh release create v1.7.3 --title "v1.7.3" --notes "Release notes..."
```

---

## 📝 Örnek Senaryolar

### Senaryo 1: Bug Fix Release (PATCH)

```bash
# Mevcut: v1.7.2
# Yeni: v1.7.3

# 1. Versiyonu güncelle
# FiratROVNet/__init__.py: __version__ = "1.7.3"

# 2. Commit
git add FiratROVNet/__init__.py
git commit -m "chore: bump version to 1.7.3"
git push origin main

# 3. Tag
git tag -a v1.7.3 -m "Release v1.7.3: Bug fixes"
git push origin v1.7.3
```

### Senaryo 2: Yeni Özellik Release (MINOR)

```bash
# Mevcut: v1.7.2
# Yeni: v1.8.0

# 1. Versiyonu güncelle
# FiratROVNet/__init__.py: __version__ = "1.8.0"

# 2. Commit
git add FiratROVNet/__init__.py
git commit -m "chore: bump version to 1.8.0"
git push origin main

# 3. Tag
git tag -a v1.8.0 -m "Release v1.8.0: Yeni özellikler eklendi"
git push origin v1.8.0
```

### Senaryo 3: Major Release (MAJOR)

```bash
# Mevcut: v1.7.2
# Yeni: v2.0.0

# 1. Versiyonu güncelle
# FiratROVNet/__init__.py: __version__ = "2.0.0"

# 2. Commit
git add FiratROVNet/__init__.py
git commit -m "chore: bump version to 2.0.0"
git push origin main

# 3. Tag
git tag -a v2.0.0 -m "Release v2.0.0: Major update - API değişiklikleri"
git push origin v2.0.0
```

---

## ✅ Best Practices

1. **Annotated Tag Kullan**: Mesajlı tag'lar daha bilgilendiricidir
2. **Semantic Versioning**: Versiyon numaralarını anlamlı şekilde artır
3. **Release Notes**: Her release için detaylı notlar yaz
4. **Test Et**: Release öncesi mutlaka test et
5. **Changelog**: Önemli değişiklikleri dokümante et

---

## 🔗 Faydalı Komutlar

```bash
# Son tag'ı göster
git describe --tags --abbrev=0

# Tag'lar arası değişiklikleri görüntüle
git log v1.7.2..v1.7.3 --oneline

# Belirli bir tag'a geri dön
git checkout v1.7.2

# Tag'ı branch'e çevir
git checkout -b release-v1.7.3 v1.7.3
```

---

## 📚 Ek Kaynaklar

- [Semantic Versioning](https://semver.org/)
- [Git Tagging](https://git-scm.com/book/en/v2/Git-Basics-Tagging)
- [GitHub Releases](https://docs.github.com/en/repositories/releasing-projects-on-github)

---

**Son Güncelleme**: 2024
**Versiyon**: 1.0

