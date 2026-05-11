"""
Takip/Formasyon Tanı Scripti
=============================
Simülasyon konsolunda çalıştır (i → tab → enter ile açılan Python konsolda):
    exec(open('takip_tani.py').read())
    takip_tani(filo)

Ya da ana filo nesnesine erişimin varsa:
    takip_tani(filo)
"""

import math


def takip_tani(filo):
    W = 65
    print("=" * W)
    print("🔍  TAKİP / FORMASYON TANI RAPORU")
    print("=" * W)

    af = getattr(filo, "aktif_formasyon", None)
    yp = getattr(filo, "yeni_pozisyonlar", None)
    rh = getattr(filo, "_rov_hedefleri", {})

    # ── 1. aktif_formasyon ────────────────────────────────────────
    print("\n[1] aktif_formasyon:")
    if af is None or af == {}:
        print("    ⚠️  Boş veya None → formasyon butonu hiç basılmamış!")
    elif not isinstance(af, dict):
        print(f"    ❌  Yanlış tip: {type(af)} → {af}")
    else:
        int_keys = [k for k in af if isinstance(k, int)]
        str_keys = [k for k in af if isinstance(k, str)]
        if str_keys and not int_keys:
            print("    ❌  FLAT DICT (eski API yazılmış)! String anahtarlar var:")
            for k, v in af.items():
                print(f"         '{k}': {v}")
            print("    ⚠️  _formasyon_dinamik_guncelle .get(group_id) sorgusu False döner → takip kırılır!")
        else:
            for k, v in af.items():
                if isinstance(k, int):
                    print(f"    ✅  Grup-{k}: {v}")
                else:
                    print(f"    ⚠️  '{k}': {v}  (string anahtar — mixed structure?)")

    # ── 2. g_rovs ────────────────────────────────────────────────
    print("\n[2] g_rovs (grup → ROV eşlemesi):")
    try:
        g_rovs = filo.g_rovs
    except Exception as e:
        print(f"    ❌ g_rovs alınamadı: {e}")
        g_rovs = {}

    if not g_rovs:
        print("    ⚠️  Boş! Hiçbir ROV group_id atanmamış veya cache rebuild edilmemiş.")
    else:
        for g_id, rovs in g_rovs.items():
            liderler   = [r for r in rovs if r and getattr(r, "role", -1) == 1]
            takipciler = [r for r in rovs if r and getattr(r, "role", -1) == 0]
            print(f"    Grup-{g_id}: toplam={len(rovs)} | "
                  f"lider={[r.id for r in liderler]} | "
                  f"takipçi={[r.id for r in takipciler]}")

    # ── 3. Her ROV detayı ────────────────────────────────────────
    print("\n[3] ROV Detay:")
    all_rovs = getattr(filo, "rovs", [])
    for r in all_rovs:
        gid  = getattr(r, "group_id", "?")
        role = getattr(r, "role", "?")
        gnc  = getattr(r, "gnc", None)
        mod  = getattr(gnc, "mod", "NO_GNC") if gnc is not None else "NO_GNC"

        # aktif_formasyon bu grup için var mı?
        if isinstance(af, dict):
            af_val = af.get(gid, None)
        else:
            af_val = None

        # Beklenen: takipçi → role=0, gnc.mod=1, af_val set
        if role == 0 and gid != 0:
            ok_role = True
            ok_mod  = (mod == 1)
            ok_af   = (af_val is not None)
            icon = "✅" if (ok_role and ok_mod and ok_af) else "❌"
            problems = []
            if not ok_mod: problems.append(f"gnc.mod={mod} (1 olmalı)")
            if not ok_af:  problems.append(f"aktif_formasyon[{gid}] yok")
            prob_str = " | ".join(problems) if problems else ""
            print(f"    {icon}  ROV-{r.id} [TAKİPÇİ]: group_id={gid}, role={role}, "
                  f"gnc.mod={mod}, af={af_val}  {prob_str}")
        elif role == 1:
            print(f"    🔹  ROV-{r.id} [LİDER   ]: group_id={gid}, gnc.mod={mod}")
        else:
            print(f"    ⬜  ROV-{r.id} [ÜSTE    ]: group_id={gid}, role={role}, gnc.mod={mod}")

    # ── 4. yeni_pozisyonlar ──────────────────────────────────────
    print(f"\n[4] yeni_pozisyonlar (lider her frame günceller):")
    if yp is None:
        print("    ⚠️  None → lider henüz pozisyon hesaplamadı (aktif_formasyon yeni mi set edildi?)")
    elif isinstance(yp, dict):
        if not yp:
            print("    ⚠️  Boş dict → lider bir kez çalışmadı mı?")
        for g_id, poz in yp.items():
            print(f"    Grup-{g_id}: {poz}")
    else:
        print(f"    ℹ️  {yp}")

    # ── 5. _rov_hedefleri ────────────────────────────────────────
    print(f"\n[5] _rov_hedefleri (takipçilere set edilen anlık hedefler):")
    if not rh:
        print("    ⚠️  Boş → takipçilere henüz hedef gönderilmemiş")
    else:
        for rid, h in rh.items():
            print(f"    ROV-{rid}: {h}")

    # ── Özet ────────────────────────────────────────────────────
    print("\n" + "=" * W)
    print("📋  ÖZET VE ÖNERİLER:")
    issues = []

    # aktif_formasyon kontrolü
    if not isinstance(af, dict):
        issues.append("❌ aktif_formasyon dict değil → formasyon butonu çalışmıyor")
    elif all(isinstance(k, str) for k in af):
        issues.append(
            "❌ aktif_formasyon FLAT DICT (eski API). "
            "Çözüm: formasyon butonuna tekrar basın (yeni kod group-keyed yazıyor)"
        )
    else:
        for g_id, rovs in g_rovs.items():
            takipciler = [r for r in rovs if r and getattr(r, "role", -1) == 0]
            if takipciler and af.get(g_id) is None:
                issues.append(
                    f"❌ Grup-{g_id} takipçi içeriyor ama aktif_formasyon[{g_id}] yok. "
                    f"Grup formasyon butonuna basın."
                )

    # gnc.mod kontrolü
    for r in all_rovs:
        role = getattr(r, "role", -1)
        gid  = getattr(r, "group_id", 0)
        gnc  = getattr(r, "gnc", None)
        mod  = getattr(gnc, "mod", 1) if gnc else 1
        if role == 0 and gid != 0 and mod == 0:
            issues.append(
                f"❌ ROV-{r.id} takipçi ama gnc.mod=0 (dondurulmuş). "
                f"Çözüm: konsoldan setattr(filo.find_rov_by_id({r.id}).gnc,'mod',1)"
            )

    # yeni_pozisyonlar kontrolü
    if af and isinstance(af, dict) and any(isinstance(k, int) for k in af):
        if not yp:
            issues.append(
                "⚠️  aktif_formasyon set ama yeni_pozisyonlar boş. "
                "Lider _formasyon_dinamik_guncelle'yi çalıştırmış olmayabilir. "
                "Bir-iki saniye bekle ve tekrar çalıştır."
            )

    if not issues:
        print("    ✅ Her şey doğru görünüyor!")
        print("    → Sorun başka yerde olabilir. gnc_helper/control.py _guncelle_hareket_uygula takip edin.")
    else:
        for iss in issues:
            print(f"    {iss}")

    print("=" * W + "\n")


# Eğer doğrudan çalıştırılırsa (konsol dışı test):
if __name__ == "__main__":
    print("Bu script simülasyon konsolunda çalıştırılmalı.")
    print("Kullanım:  exec(open('takip_tani.py').read()); takip_tani(filo)")
