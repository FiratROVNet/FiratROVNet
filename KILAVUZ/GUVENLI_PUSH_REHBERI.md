# ✅ Güvenli Push Rehberi (Zorunlu Adımlar)

Bu doküman, **organizasyon GitHub repolarında** güvenli şekilde  
**commit atmak ve push yapmak** için izlenmesi gereken **zorunlu adımları** içerir.

> ⚠️ Bu rehber **force push içermez** ve ekip çalışmasına uygundur.

---

## 1️⃣ Doğru Branch ve Remote Kontrolü

Öncelikle doğru branch’te ve doğru remote’a bağlı olduğundan emin ol:

```bash
git branch
git remote -v


Beklenen çıktı:

* main
origin https://github.com/FiratROVNet/FiratROVNet.git

2️⃣ Remote’daki Güncel Durumu Al

Remote repository’deki son değişiklikleri local’e çekmeden önce mutlaka kontrol et:

git fetch origin

3️⃣ Değişiklikleri Stage Et

Yaptığın tüm değişiklikleri commit’e hazır hale getir:

git add .
git status


git status çıktısını mutlaka kontrol et.
Gereksiz dosyaların (ör. __pycache__) eklenmediğinden emin ol.

4️⃣ Commit At (Zorunlu)

Anlamlı ve açıklayıcı bir commit mesajı kullan:

git commit -m "fix: improve simulation logic and add documentation"


Commit mesajları mümkünse Conventional Commits formatında olmalıdır.

5️⃣ Remote ile Senkron Ol (EN KRİTİK ADIM)

Push atmadan önce remote’daki değişikliklerle local commit’lerini rebase et:

git pull --rebase origin main

Çakışma Olursa
git status
# çakışan dosyayı düzelt
git add <dosya>
git rebase --continue


❗ Bu adım atlanırsa push işlemi reddedilebilir.

6️⃣ Güvenli Push

Her şey senkron ise artık güvenle push atabilirsin:

git push origin main

🧾 Hızlı Özet (Kopyala–Yapıştır)
git branch
git remote -v
git fetch origin
git add .
git commit -m "fix: improve simulation logic and add documentation"
git pull --rebase origin main
git push origin main

❌ Asla Yapılmaması Gerekenler
git push -f
git pull origin main


Bu komutlar organizasyon repo’larında veri kaybına yol açabilir.

✅ Sonuç

✔ Repository güvenliği korunur

✔ Commit geçmişi temiz kalır

✔ Ekip çalışmasına uygundur
