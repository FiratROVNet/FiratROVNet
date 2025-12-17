# 🌿 Git Flow Rehberi - FıratROVNet

Bu rehber, FıratROVNet projesinde **Git Flow** yapısını kullanarak güvenli ve organize bir şekilde geliştirme yapmanızı sağlar.

---

## 📋 İçindekiler

1. [Git Flow Nedir?](#git-flow-nedir)
2. [Branch Yapısı](#branch-yapısı)
3. [Temel Kullanım](#temel-kullanım)
4. [Workflow Senaryoları](#workflow-senaryoları)
5. [Best Practices](#best-practices)
6. [Hızlı Referans](#hızlı-referans)

---

## 🌳 Git Flow Nedir?

Git Flow, yazılım geliştirme süreçlerini organize etmek için kullanılan bir **branch stratejisidir**. Farklı amaçlar için farklı branch'ler kullanarak:

- ✅ **Güvenli geliştirme** (main branch korunur)
- ✅ **Paralel çalışma** (birden fazla özellik aynı anda)
- ✅ **Kolay geri alma** (hata durumunda)
- ✅ **Düzenli release'ler** (kontrollü yayınlama)

sağlar.

---

## 🌿 Branch Yapısı

### Ana Branch'ler

```
main        → Canlı sürüm (production-ready kod)
develop     → Geliştirme (tüm özellikler burada birleşir)
```

### Destekleyici Branch'ler

```
feature/*   → Yeni özellikler (feature/sonar-sistemi)
bugfix/*    → Hata düzeltmeleri (bugfix/batarya-hatasi)
release/*   → Yayına hazırlık (release/v1.8.0)
hotfix/*    → Canlı acil düzeltme (hotfix/kritik-hata)
```

---

## 🚀 Temel Kullanım

### 1. İlk Kurulum

```bash
# Repository'yi klonla
git clone https://github.com/FiratROVNet/FiratROVNet.git
cd FiratROVNet

# develop branch'ine geç
git checkout develop
git pull origin develop
```

### 2. Yeni Özellik Geliştirme (Feature)

```bash
# 1. develop'den yeni feature branch oluştur
git checkout develop
git pull origin develop
git checkout -b feature/sonar-guncellemesi

# 2. Kodunu yaz, commit at
git add .
git commit -m "feat: sonar sistemi güncellemesi"

# 3. develop'e merge et
git checkout develop
git pull origin develop
git merge feature/sonar-guncellemesi

# 4. develop'i push et
git push origin develop

# 5. Feature branch'i sil (opsiyonel)
git branch -d feature/sonar-guncellemesi
git push origin --delete feature/sonar-guncellemesi
```

### 3. Hata Düzeltme (Bugfix)

```bash
# 1. develop'den bugfix branch oluştur
git checkout develop
git pull origin develop
git checkout -b bugfix/batarya-hatasi

# 2. Hatayı düzelt, commit at
git add .
git commit -m "fix: batarya hesaplama hatası düzeltildi"

# 3. develop'e merge et
git checkout develop
git pull origin develop
git merge bugfix/batarya-hatasi

# 4. develop'i push et
git push origin develop
```

### 4. Release Hazırlığı

```bash
# 1. develop'den release branch oluştur
git checkout develop
git pull origin develop
git checkout -b release/v1.8.0

# 2. Versiyonu güncelle (__init__.py)
# __version__ = "1.8.0"

# 3. Son testler ve düzeltmeler
git add .
git commit -m "chore: bump version to 1.8.0"

# 4. main'e merge et
git checkout main
git pull origin main
git merge release/v1.8.0

# 5. Tag oluştur ve push et
git tag -a v1.8.0 -m "Release v1.8.0"
git push origin main
git push origin v1.8.0

# 6. develop'e de merge et (release değişikliklerini geri al)
git checkout develop
git merge release/v1.8.0
git push origin develop

# 7. Release branch'i sil
git branch -d release/v1.8.0
```

### 5. Acil Düzeltme (Hotfix)

```bash
# 1. main'den hotfix branch oluştur
git checkout main
git pull origin main
git checkout -b hotfix/kritik-guvenlik-hatasi

# 2. Acil düzeltmeyi yap
git add .
git commit -m "fix: kritik güvenlik hatası düzeltildi"

# 3. main'e merge et (hızlı!)
git checkout main
git merge hotfix/kritik-guvenlik-hatasi

# 4. Versiyonu güncelle (PATCH artır: 1.7.3 → 1.7.4)
# __version__ = "1.7.4"
git add .
git commit -m "chore: bump version to 1.7.4"

# 5. Tag oluştur ve push et
git tag -a v1.7.4 -m "Hotfix v1.7.4: Kritik güvenlik hatası"
git push origin main
git push origin v1.7.4

# 6. develop'e de merge et
git checkout develop
git merge hotfix/kritik-guvenlik-hatasi
git push origin develop

# 7. Hotfix branch'i sil
git branch -d hotfix/kritik-guvenlik-hatasi
```

---

## 📖 Workflow Senaryoları

### Senaryo 1: Yeni Özellik Ekleme

**Durum**: Sonar sistemi için yeni bir özellik eklemek istiyorsunuz.

```bash
# 1. develop'den feature branch oluştur
git checkout develop
git pull origin develop
git checkout -b feature/sonar-gelismeleri

# 2. Özelliği geliştir
# ... kod yaz ...

# 3. Commit at
git add .
git commit -m "feat: sonar sistemi için yeni algılama algoritması"

# 4. develop'e merge et
git checkout develop
git pull origin develop
git merge feature/sonar-gelismeleri

# 5. Test et (develop'de)
# ... testler ...

# 6. Push et
git push origin develop

# 7. Feature branch'i sil
git branch -d feature/sonar-gelismeleri
```

### Senaryo 2: Release Oluşturma

**Durum**: v1.8.0 release'ini hazırlamak istiyorsunuz.

```bash
# 1. develop'den release branch oluştur
git checkout develop
git pull origin develop
git checkout -b release/v1.8.0

# 2. Versiyonu güncelle
# FiratROVNet/__init__.py: __version__ = "1.8.0"
git add FiratROVNet/__init__.py
git commit -m "chore: bump version to 1.8.0"

# 3. Son testler ve dokümantasyon güncellemeleri
# ... son kontroller ...

# 4. main'e merge et
git checkout main
git pull origin main
git merge release/v1.8.0

# 5. Tag oluştur
git tag -a v1.8.0 -m "Release v1.8.0: Yeni özellikler"
git push origin main
git push origin v1.8.0

# 6. GitHub release oluştur
gh release create v1.8.0 --title "v1.8.0" --notes "Release notes..."

# 7. develop'e geri merge et
git checkout develop
git merge release/v1.8.0
git push origin develop

# 8. Release branch'i sil
git branch -d release/v1.8.0
```

### Senaryo 3: Acil Hotfix

**Durum**: Canlı sistemde kritik bir hata var, hemen düzeltilmeli.

```bash
# 1. main'den hotfix branch oluştur
git checkout main
git pull origin main
git checkout -b hotfix/kritik-hata

# 2. Hatayı düzelt
# ... hata düzeltmesi ...

# 3. Commit at
git add .
git commit -m "fix: kritik hata düzeltildi"

# 4. main'e merge et
git checkout main
git merge hotfix/kritik-hata

# 5. Versiyonu güncelle (PATCH: 1.7.3 → 1.7.4)
# __version__ = "1.7.4"
git add .
git commit -m "chore: bump version to 1.7.4"

# 6. Tag ve push
git tag -a v1.7.4 -m "Hotfix v1.7.4"
git push origin main
git push origin v1.7.4

# 7. develop'e de merge et
git checkout develop
git merge hotfix/kritik-hata
git push origin develop

# 8. Hotfix branch'i sil
git branch -d hotfix/kritik-hata
```

---

## ✅ Best Practices

### 1. Branch İsimlendirme

```bash
# ✅ İyi
feature/sonar-sistemi
bugfix/batarya-hesaplama
release/v1.8.0
hotfix/kritik-hata

# ❌ Kötü
feature1
bug
release
fix
```

### 2. Commit Mesajları

**Conventional Commits** formatını kullanın:

```bash
# ✅ İyi
feat: sonar sistemi güncellemesi
fix: batarya hesaplama hatası
chore: versiyon güncellemesi
docs: README güncellemesi
refactor: kod organizasyonu

# ❌ Kötü
değişiklik
hata düzeltme
güncelleme
```

### 3. Merge Öncesi Kontroller

```bash
# 1. Güncel develop'i çek
git checkout develop
git pull origin develop

# 2. Feature branch'i güncelle
git checkout feature/yeni-ozellik
git merge develop  # veya rebase

# 3. Test et
# ... testler ...

# 4. Merge et
git checkout develop
git merge feature/yeni-ozellik
```

### 4. Branch Temizliği

```bash
# Kullanılmayan branch'leri sil
git branch -d feature/eski-ozellik
git push origin --delete feature/eski-ozellik

# Tüm merge edilmiş branch'leri listele
git branch --merged develop
```

---

## 🚨 Önemli Kurallar

### ❌ ASLA YAPMAYIN

1. **main branch'e direkt commit atmayın**
   - ❌ `git checkout main && git commit`
   - ✅ Feature/bugfix branch kullanın

2. **Force push yapmayın**
   - ❌ `git push -f`
   - ✅ Normal push kullanın

3. **develop'i atlamayın**
   - ❌ Feature → main (direkt)
   - ✅ Feature → develop → main

4. **Release branch'i uzun süre tutmayın**
   - Release hazır olduğunda hemen merge edin

### ✅ YAPIN

1. **Her zaman develop'den başlayın**
2. **Feature branch'leri kısa tutun**
3. **Merge öncesi test edin**
4. **Commit mesajlarını açıklayıcı yazın**
5. **Branch'leri temiz tutun**

---

## 📊 Branch Yaşam Döngüsü

```
feature/*  → develop → main
bugfix/*   → develop → main
release/*  → main + develop
hotfix/*   → main + develop
```

---

## 🔧 Hızlı Referans

### Branch Oluşturma

```bash
# Feature
git checkout -b feature/isim develop

# Bugfix
git checkout -b bugfix/isim develop

# Release
git checkout -b release/vX.Y.Z develop

# Hotfix
git checkout -b hotfix/isim main
```

### Merge İşlemleri

```bash
# Feature → develop
git checkout develop
git merge feature/isim

# Bugfix → develop
git checkout develop
git merge bugfix/isim

# Release → main
git checkout main
git merge release/vX.Y.Z

# Hotfix → main
git checkout main
git merge hotfix/isim
```

### Branch Silme

```bash
# Local branch sil
git branch -d branch-ismi

# Remote branch sil
git push origin --delete branch-ismi
```

---

## 📚 İlgili Dokümantasyon

- [Release Versiyon Yönetimi](RELEASE_VERSIYON_YONETIMI.md)
- [Güvenli Push Rehberi](GUVENLI_PUSH_REHBERI.md)
- [Conventional Commits](https://www.conventionalcommits.org/)

---

## 🆘 Sorun Giderme

### Merge Conflict

```bash
# Conflict çözümü
git merge feature/isim
# ... conflict'leri düzelt ...
git add .
git commit
```

### Yanlış Branch'e Commit

```bash
# Commit'i taşı
git log --oneline
git cherry-pick <commit-hash>
git checkout dogru-branch
git cherry-pick <commit-hash>
```

### Branch'i Geri Alma

```bash
# Son commit'i geri al (soft)
git reset --soft HEAD~1

# Son commit'i geri al (hard - dikkatli!)
git reset --hard HEAD~1
```

---

**Son Güncelleme**: 2024  
**Versiyon**: 1.0
