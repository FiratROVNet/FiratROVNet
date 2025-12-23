# 🔀 Pull Request (PR) Workflow Rehberi

Branch protection kuralları aktif olduğu için artık `main` branch'ine **direkt push yapılamaz**. Tüm değişiklikler **Pull Request (PR)** ile yapılmalıdır.

---

## 📋 İçindekiler

1. [Temel Workflow](#temel-workflow)
2. [Günlük Geliştirme (Feature)](#günlük-geliştirme-feature)
3. [Release Oluşturma](#release-oluşturma)
4. [Hotfix (Acil Düzeltme)](#hotfix-acil-düzeltme)
5. [PR Oluşturma Adımları](#pr-oluşturma-adımları)
6. [Sık Sorulan Sorular](#sık-sorulan-sorular)

---

## 🔄 Temel Workflow

```
┌─────────────┐
│   develop   │ ← Günlük geliştirmeler burada
└──────┬──────┘
       │
       ├─── feature/* ────┐
       │                   │
       ├─── bugfix/* ──────┤ → PR → develop
       │                   │
       └─── release/* ─────┘
                           │
                           ↓
                    ┌─────────────┐
                    │    main    │ ← Sadece release/hotfix (PR ile)
                    └────────────┘
```

---

## 🚀 Günlük Geliştirme (Feature)

### Senaryo: Yeni bir özellik eklemek istiyorsunuz

#### 1. Feature Branch Oluştur

```bash
# develop branch'ine geç ve güncelle
git checkout develop
git pull origin develop

# Yeni feature branch oluştur
git checkout -b feature/sonar-gelismeleri

# Kodunu yaz...
```

#### 2. Commit ve Push

```bash
# Değişiklikleri commit et
git add .
git commit -m "feat: sonar sistemi için yeni algılama algoritması"

# Feature branch'i remote'a push et
git push origin feature/sonar-gelismeleri
```

#### 3. Pull Request Oluştur

**GitHub Web Arayüzü:**

1. GitHub repository'ye git: `https://github.com/FiratROVNet/FiratROVNet`
2. "Compare & pull request" butonuna tıkla (veya "Pull requests" → "New pull request")
3. **Base branch**: `develop` seç
4. **Compare branch**: `feature/sonar-gelismeleri` seç
5. PR başlığı ve açıklama yaz:

```markdown
## 🎯 Amaç
Sonar sistemi için yeni algılama algoritması eklendi.

## ✨ Değişiklikler
- Yeni algılama algoritması
- Performans iyileştirmeleri

## 🧪 Test
- [x] Manuel test yapıldı
- [x] Kod review yapıldı
```

6. "Create pull request" butonuna tıkla

#### 4. Review ve Merge

- PR oluşturulduktan sonra review bekler
- Review onaylandıktan sonra "Merge pull request" butonuna tıkla
- Feature branch otomatik olarak `develop`'e merge edilir

#### 5. Temizlik

```bash
# Local feature branch'i sil
git checkout develop
git pull origin develop
git branch -d feature/sonar-gelismeleri
```

---

## 🏷️ Release Oluşturma

### Senaryo: v1.8.0 release'ini hazırlamak istiyorsunuz

#### 1. Release Branch Oluştur

```bash
# develop'den release branch oluştur
git checkout develop
git pull origin develop
git checkout -b release/v1.8.0

# Versiyonu güncelle
# FiratROVNet/__init__.py: __version__ = "1.8.0"
git add FiratROVNet/__init__.py
git commit -m "chore: bump version to 1.8.0"

# Son testler ve dokümantasyon güncellemeleri
# ... testler ...

# Release branch'i push et
git push origin release/v1.8.0
```

#### 2. PR Oluştur (release → main)

**GitHub Web Arayüzü:**

1. "New pull request" → "Compare & pull request"
2. **Base branch**: `main` seç
3. **Compare branch**: `release/v1.8.0` seç
4. PR başlığı: `Release v1.8.0`
5. PR açıklaması:

```markdown
## 🎉 Release v1.8.0

### ✨ Yeni Özellikler
- Sonar sistemi güncellemesi
- Yeni algılama algoritması

### 🐛 Hata Düzeltmeleri
- Batarya hesaplama hatası düzeltildi

### 📝 Değişiklikler
- Kod refactoring
- Dokümantasyon güncellemeleri

**Ready for production** ✅
```

6. "Create pull request" → Review → Merge

#### 3. Tag Oluştur

PR merge edildikten sonra:

```bash
# main branch'ini güncelle
git checkout main
git pull origin main

# Tag oluştur
git tag -a v1.8.0 -m "Release v1.8.0: Yeni özellikler"
git push origin v1.8.0

# GitHub release oluştur
gh release create v1.8.0 --title "v1.8.0" --notes "Release notes..."
```

#### 4. Develop'e Geri Merge

```bash
# Release değişikliklerini develop'e de al
git checkout develop
git pull origin develop
git merge release/v1.8.0
git push origin develop

# Release branch'i sil
git branch -d release/v1.8.0
git push origin --delete release/v1.8.0
```

---

## 🔥 Hotfix (Acil Düzeltme)

### Senaryo: Canlı sistemde kritik bir hata var

#### 1. Hotfix Branch Oluştur

```bash
# main'den hotfix branch oluştur
git checkout main
git pull origin main
git checkout -b hotfix/kritik-hata

# Hatayı düzelt
# ... düzeltme ...

git add .
git commit -m "fix: kritik hata düzeltildi"

# Versiyonu güncelle (PATCH: 1.7.3 → 1.7.4)
# __version__ = "1.7.4"
git add FiratROVNet/__init__.py
git commit -m "chore: bump version to 1.7.4"

# Push et
git push origin hotfix/kritik-hata
```

#### 2. PR Oluştur (hotfix → main)

**GitHub Web Arayüzü:**

1. "New pull request"
2. **Base branch**: `main`
3. **Compare branch**: `hotfix/kritik-hata`
4. PR başlığı: `Hotfix v1.7.4: Kritik hata düzeltmesi`
5. PR açıklaması:

```markdown
## 🔥 Acil Hotfix

**Kritik hata düzeltmesi - Hemen merge edilmeli**

### 🐛 Hata
- [Açıklama]

### ✅ Düzeltme
- [Açıklama]

**Priority: HIGH** ⚠️
```

6. Review → Merge (hızlı!)

#### 3. Tag ve Release

```bash
git checkout main
git pull origin main
git tag -a v1.7.4 -m "Hotfix v1.7.4"
git push origin v1.7.4
gh release create v1.7.4 --title "v1.7.4" --notes "Hotfix"
```

#### 4. Develop'e Merge

```bash
git checkout develop
git merge hotfix/kritik-hata
git push origin develop
```

---

## 📝 PR Oluşturma Adımları (Detaylı)

### GitHub Web Arayüzü

1. **Repository'ye Git**
   ```
   https://github.com/FiratROVNet/FiratROVNet
   ```

2. **"Pull requests" Tab'ına Tıkla**

3. **"New pull request" Butonuna Tıkla**

4. **Branch'leri Seç**
   - **base**: Hedef branch (örn: `main` veya `develop`)
   - **compare**: Kaynak branch (örn: `feature/yeni-ozellik`)

5. **PR Bilgilerini Doldur**
   - **Title**: Açıklayıcı başlık
   - **Description**: Detaylı açıklama

6. **Reviewer Ekle** (opsiyonel)
   - "Reviewers" bölümünden reviewer ekle

7. **"Create pull request" Butonuna Tıkla**

### GitHub CLI (gh)

```bash
# PR oluştur
gh pr create \
  --base develop \
  --head feature/yeni-ozellik \
  --title "feat: yeni özellik" \
  --body "Açıklama..."

# PR listele
gh pr list

# PR detaylarını görüntüle
gh pr view <PR_NUMBER>

# PR merge et (otomatik)
gh pr merge <PR_NUMBER> --merge
```

---

## ❓ Sık Sorulan Sorular

### Q: main'e direkt push yapabilir miyim?

**A:** ❌ Hayır. Branch protection kuralları nedeniyle main'e sadece PR ile merge edebilirsiniz.

### Q: develop'e direkt push yapabilir miyim?

**A:** ✅ Evet, develop branch'ine direkt push yapabilirsiniz. Ancak önerilen workflow:
- Feature branch oluştur
- PR ile develop'e merge et

### Q: PR merge edildikten sonra ne yapmalıyım?

**A:** 
1. Local branch'leri güncelle:
   ```bash
   git checkout main  # veya develop
   git pull origin main
   ```
2. Feature branch'i sil:
   ```bash
   git branch -d feature/isim
   ```

### Q: PR'ı kim merge edebilir?

**A:** 
- Repository owner
- Maintainer
- Review onayı alan kişiler (ayarlara göre)

### Q: PR merge edilmeden önce test yapabilir miyim?

**A:** ✅ Evet, PR'ı local'de test edebilirsiniz:
```bash
# PR branch'ini local'e çek
git fetch origin
git checkout feature/yeni-ozellik
```

### Q: PR'da conflict varsa ne yapmalıyım?

**A:**
1. Conflict'leri çöz:
   ```bash
   git checkout feature/yeni-ozellik
   git pull origin develop  # veya main
   # Conflict'leri düzelt
   git add .
   git commit -m "fix: merge conflict resolved"
   git push origin feature/yeni-ozellik
   ```
2. PR otomatik olarak güncellenir

---

## 🎯 Özet: Hangi Durumda Ne Yapmalı?

| Durum | Branch | PR → | Notlar |
|-------|-------|------|--------|
| Yeni özellik | `feature/*` | `develop` | Günlük geliştirme |
| Hata düzeltme | `bugfix/*` | `develop` | Normal hata düzeltme |
| Release hazırlık | `release/*` | `main` | Versiyon güncellemesi |
| Acil düzeltme | `hotfix/*` | `main` | Kritik hata |

---

## ✅ Best Practices

1. **Her zaman feature branch kullan**
   - ❌ `git push origin develop` (direkt)
   - ✅ `git push origin feature/isim` → PR

2. **PR açıklamalarını detaylı yaz**
   - Ne değişti?
   - Neden değişti?
   - Nasıl test edildi?

3. **Küçük PR'lar tercih et**
   - Büyük PR'lar review'ı zorlaştırır
   - Her PR tek bir özellik/hata düzeltmesi

4. **PR'ı merge etmeden önce test et**
   - Local'de test et
   - CI/CD testlerinin geçmesini bekle

5. **Merge sonrası temizlik yap**
   - Feature branch'leri sil
   - Local branch'leri güncelle

---

## 🔗 İlgili Dokümantasyon

- [Git Flow Rehberi](GIT_FLOW_REHBERI.md)
- [Release Versiyon Yönetimi](RELEASE_VERSIYON_YONETIMI.md)
- [Güvenli Push Rehberi](GUVENLI_PUSH_REHBERI.md)

---

**Son Güncelleme**: 2024  
**Versiyon**: 1.0








# 🔀 Pull Request (PR) Workflow Rehberi

Branch protection kuralları aktif olduğu için artık `main` branch'ine **direkt push yapılamaz**. Tüm değişiklikler **Pull Request (PR)** ile yapılmalıdır.

---

## 📋 İçindekiler

1. [Temel Workflow](#temel-workflow)
2. [Günlük Geliştirme (Feature)](#günlük-geliştirme-feature)
3. [Release Oluşturma](#release-oluşturma)
4. [Hotfix (Acil Düzeltme)](#hotfix-acil-düzeltme)
5. [PR Oluşturma Adımları](#pr-oluşturma-adımları)
6. [Sık Sorulan Sorular](#sık-sorulan-sorular)

---

## 🔄 Temel Workflow

```
┌─────────────┐
│   develop   │ ← Günlük geliştirmeler burada
└──────┬──────┘
       │
       ├─── feature/* ────┐
       │                   │
       ├─── bugfix/* ──────┤ → PR → develop
       │                   │
       └─── release/* ─────┘
                           │
                           ↓
                    ┌─────────────┐
                    │    main    │ ← Sadece release/hotfix (PR ile)
                    └────────────┘
```

---

## 🚀 Günlük Geliştirme (Feature)

### Senaryo: Yeni bir özellik eklemek istiyorsunuz

#### 1. Feature Branch Oluştur

```bash
# develop branch'ine geç ve güncelle
git checkout develop
git pull origin develop

# Yeni feature branch oluştur
git checkout -b feature/sonar-gelismeleri

# Kodunu yaz...
```

#### 2. Commit ve Push

```bash
# Değişiklikleri commit et
git add .
git commit -m "feat: sonar sistemi için yeni algılama algoritması"

# Feature branch'i remote'a push et
git push origin feature/sonar-gelismeleri
```

#### 3. Pull Request Oluştur

**GitHub Web Arayüzü:**

1. GitHub repository'ye git: `https://github.com/FiratROVNet/FiratROVNet`
2. "Compare & pull request" butonuna tıkla (veya "Pull requests" → "New pull request")
3. **Base branch**: `develop` seç
4. **Compare branch**: `feature/sonar-gelismeleri` seç
5. PR başlığı ve açıklama yaz:

```markdown
## 🎯 Amaç
Sonar sistemi için yeni algılama algoritması eklendi.

## ✨ Değişiklikler
- Yeni algılama algoritması
- Performans iyileştirmeleri

## 🧪 Test
- [x] Manuel test yapıldı
- [x] Kod review yapıldı
```

6. "Create pull request" butonuna tıkla

#### 4. Review ve Merge

- PR oluşturulduktan sonra review bekler
- Review onaylandıktan sonra "Merge pull request" butonuna tıkla
- Feature branch otomatik olarak `develop`'e merge edilir

#### 5. Temizlik

```bash
# Local feature branch'i sil
git checkout develop
git pull origin develop
git branch -d feature/sonar-gelismeleri
```

---

## 🏷️ Release Oluşturma

### Senaryo: v1.8.0 release'ini hazırlamak istiyorsunuz

#### 1. Release Branch Oluştur

```bash
# develop'den release branch oluştur
git checkout develop
git pull origin develop
git checkout -b release/v1.8.0

# Versiyonu güncelle
# FiratROVNet/__init__.py: __version__ = "1.8.0"
git add FiratROVNet/__init__.py
git commit -m "chore: bump version to 1.8.0"

# Son testler ve dokümantasyon güncellemeleri
# ... testler ...

# Release branch'i push et
git push origin release/v1.8.0
```

#### 2. PR Oluştur (release → main)

**GitHub Web Arayüzü:**

1. "New pull request" → "Compare & pull request"
2. **Base branch**: `main` seç
3. **Compare branch**: `release/v1.8.0` seç
4. PR başlığı: `Release v1.8.0`
5. PR açıklaması:

```markdown
## 🎉 Release v1.8.0

### ✨ Yeni Özellikler
- Sonar sistemi güncellemesi
- Yeni algılama algoritması

### 🐛 Hata Düzeltmeleri
- Batarya hesaplama hatası düzeltildi

### 📝 Değişiklikler
- Kod refactoring
- Dokümantasyon güncellemeleri

**Ready for production** ✅
```

6. "Create pull request" → Review → Merge

#### 3. Tag Oluştur

PR merge edildikten sonra:

```bash
# main branch'ini güncelle
git checkout main
git pull origin main

# Tag oluştur
git tag -a v1.8.0 -m "Release v1.8.0: Yeni özellikler"
git push origin v1.8.0

# GitHub release oluştur
gh release create v1.8.0 --title "v1.8.0" --notes "Release notes..."
```

#### 4. Develop'e Geri Merge

```bash
# Release değişikliklerini develop'e de al
git checkout develop
git pull origin develop
git merge release/v1.8.0
git push origin develop

# Release branch'i sil
git branch -d release/v1.8.0
git push origin --delete release/v1.8.0
```

---

## 🔥 Hotfix (Acil Düzeltme)

### Senaryo: Canlı sistemde kritik bir hata var

#### 1. Hotfix Branch Oluştur

```bash
# main'den hotfix branch oluştur
git checkout main
git pull origin main
git checkout -b hotfix/kritik-hata

# Hatayı düzelt
# ... düzeltme ...

git add .
git commit -m "fix: kritik hata düzeltildi"

# Versiyonu güncelle (PATCH: 1.7.3 → 1.7.4)
# __version__ = "1.7.4"
git add FiratROVNet/__init__.py
git commit -m "chore: bump version to 1.7.4"

# Push et
git push origin hotfix/kritik-hata
```

#### 2. PR Oluştur (hotfix → main)

**GitHub Web Arayüzü:**

1. "New pull request"
2. **Base branch**: `main`
3. **Compare branch**: `hotfix/kritik-hata`
4. PR başlığı: `Hotfix v1.7.4: Kritik hata düzeltmesi`
5. PR açıklaması:

```markdown
## 🔥 Acil Hotfix

**Kritik hata düzeltmesi - Hemen merge edilmeli**

### 🐛 Hata
- [Açıklama]

### ✅ Düzeltme
- [Açıklama]

**Priority: HIGH** ⚠️
```

6. Review → Merge (hızlı!)

#### 3. Tag ve Release

```bash
git checkout main
git pull origin main
git tag -a v1.7.4 -m "Hotfix v1.7.4"
git push origin v1.7.4
gh release create v1.7.4 --title "v1.7.4" --notes "Hotfix"
```

#### 4. Develop'e Merge

```bash
git checkout develop
git merge hotfix/kritik-hata
git push origin develop
```

---

## 📝 PR Oluşturma Adımları (Detaylı)

### GitHub Web Arayüzü

1. **Repository'ye Git**
   ```
   https://github.com/FiratROVNet/FiratROVNet
   ```

2. **"Pull requests" Tab'ına Tıkla**

3. **"New pull request" Butonuna Tıkla**

4. **Branch'leri Seç**
   - **base**: Hedef branch (örn: `main` veya `develop`)
   - **compare**: Kaynak branch (örn: `feature/yeni-ozellik`)

5. **PR Bilgilerini Doldur**
   - **Title**: Açıklayıcı başlık
   - **Description**: Detaylı açıklama

6. **Reviewer Ekle** (opsiyonel)
   - "Reviewers" bölümünden reviewer ekle

7. **"Create pull request" Butonuna Tıkla**

### GitHub CLI (gh)

```bash
# PR oluştur
gh pr create \
  --base develop \
  --head feature/yeni-ozellik \
  --title "feat: yeni özellik" \
  --body "Açıklama..."

# PR listele
gh pr list

# PR detaylarını görüntüle
gh pr view <PR_NUMBER>

# PR merge et (otomatik)
gh pr merge <PR_NUMBER> --merge
```

---

## ❓ Sık Sorulan Sorular

### Q: main'e direkt push yapabilir miyim?

**A:** ❌ Hayır. Branch protection kuralları nedeniyle main'e sadece PR ile merge edebilirsiniz.

### Q: develop'e direkt push yapabilir miyim?

**A:** ✅ Evet, develop branch'ine direkt push yapabilirsiniz. Ancak önerilen workflow:
- Feature branch oluştur
- PR ile develop'e merge et

### Q: PR merge edildikten sonra ne yapmalıyım?

**A:** 
1. Local branch'leri güncelle:
   ```bash
   git checkout main  # veya develop
   git pull origin main
   ```
2. Feature branch'i sil:
   ```bash
   git branch -d feature/isim
   ```

### Q: PR'ı kim merge edebilir?

**A:** 
- Repository owner
- Maintainer
- Review onayı alan kişiler (ayarlara göre)

### Q: PR merge edilmeden önce test yapabilir miyim?

**A:** ✅ Evet, PR'ı local'de test edebilirsiniz:
```bash
# PR branch'ini local'e çek
git fetch origin
git checkout feature/yeni-ozellik
```

### Q: PR'da conflict varsa ne yapmalıyım?

**A:**
1. Conflict'leri çöz:
   ```bash
   git checkout feature/yeni-ozellik
   git pull origin develop  # veya main
   # Conflict'leri düzelt
   git add .
   git commit -m "fix: merge conflict resolved"
   git push origin feature/yeni-ozellik
   ```
2. PR otomatik olarak güncellenir

---

## 🎯 Özet: Hangi Durumda Ne Yapmalı?

| Durum | Branch | PR → | Notlar |
|-------|-------|------|--------|
| Yeni özellik | `feature/*` | `develop` | Günlük geliştirme |
| Hata düzeltme | `bugfix/*` | `develop` | Normal hata düzeltme |
| Release hazırlık | `release/*` | `main` | Versiyon güncellemesi |
| Acil düzeltme | `hotfix/*` | `main` | Kritik hata |

---

## ✅ Best Practices

1. **Her zaman feature branch kullan**
   - ❌ `git push origin develop` (direkt)
   - ✅ `git push origin feature/isim` → PR

2. **PR açıklamalarını detaylı yaz**
   - Ne değişti?
   - Neden değişti?
   - Nasıl test edildi?

3. **Küçük PR'lar tercih et**
   - Büyük PR'lar review'ı zorlaştırır
   - Her PR tek bir özellik/hata düzeltmesi

4. **PR'ı merge etmeden önce test et**
   - Local'de test et
   - CI/CD testlerinin geçmesini bekle

5. **Merge sonrası temizlik yap**
   - Feature branch'leri sil
   - Local branch'leri güncelle

---

## 🔗 İlgili Dokümantasyon

- [Git Flow Rehberi](GIT_FLOW_REHBERI.md)
- [Release Versiyon Yönetimi](RELEASE_VERSIYON_YONETIMI.md)
- [Güvenli Push Rehberi](GUVENLI_PUSH_REHBERI.md)

---

**Son Güncelleme**: 2024  
**Versiyon**: 1.0













