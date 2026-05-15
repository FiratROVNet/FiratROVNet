"""
UI Panel Testleri — Headless (QT_QPA_PLATFORM=offscreen)
Çalıştırma: QT_QPA_PLATFORM=offscreen python -m pytest tests/test_ui_panels.py -v
           veya direkt: QT_QPA_PLATFORM=offscreen python tests/test_ui_panels.py
"""
import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# Proje kökünü sys.path'e ekle
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from PyQt5.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)


# ── Test altyapısı ────────────────────────────────────────────────────────────
_results = {"passed": [], "failed": [], "skipped": []}

def _pass(name: str):
    _results["passed"].append(name)
    print(f"  ✅ {name}")

def _fail(name: str, err):
    _results["failed"].append((name, str(err)))
    print(f"  ❌ {name}: {err}")

def _skip(name: str, reason: str = ""):
    _results["skipped"].append((name, reason))
    print(f"  ⏭  {name}{(' — ' + reason) if reason else ''}")


# ═══════════════════════════════════════════════════════════════════════════════
# BÖLÜM 1: kopru.py fonksiyon imzaları
# ═══════════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 60)
print("TEST 1: kopru.py fonksiyon imzaları")
print("=" * 60)

try:
    from UI.kopru import (
        rov_listesi, grup_bilgisi, sim_bagli_mi, bagli_mi,
        komut_gonder, filo_bagla, aktif_gorevler_bilgisi,
    )
    _pass("kopru.py imports (7 fonksiyon)")
except ImportError as e:
    _fail("kopru.py imports", e)

try:
    # bagli_mi() False iken hiçbir şey çökmemeli
    assert not bagli_mi(), "bagli_mi() aynı-process bağlantı yokken False olmalı"
    # sim_bagli_mi() ve rov_listesi() çağrıları exception atmamali
    _ = sim_bagli_mi()
    _ = rov_listesi()
    _ = grup_bilgisi()
    _ = aktif_gorevler_bilgisi()
    assert aktif_gorevler_bilgisi() == {}, "aktif_gorevler_bilgisi() bağlantısız {} dönmeli"
    _pass("kopru.py bağlantısız güvenlik")
except Exception as e:
    _fail("kopru.py bağlantısız güvenlik", e)


# ═══════════════════════════════════════════════════════════════════════════════
# BÖLÜM 2: GorevPanel — widget oluşturma + sekme sayısı
# ═══════════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 60)
print("TEST 2: GorevPanel widget oluşturma")
print("=" * 60)

try:
    from UI.paneller.gorev_panel import (
        AlanTaramaSekmesi, AramaKurtarmaSekmesi,
        ImhaSekmesi, DurumSekmesi, HareketSekmesi, GorevPanel,
    )
    _pass("gorev_panel imports")
except ImportError as e:
    _fail("gorev_panel imports", e)
    sys.exit(1)

# AlanTaramaSekmesi
try:
    at = AlanTaramaSekmesi(None)
    assert hasattr(at, "spin_grup"), "spin_grup yok"
    assert hasattr(at, "spin_derinlik"), "spin_derinlik yok"
    assert hasattr(at, "spin_serit"), "spin_serit yok"
    assert hasattr(at, "spin_rov_say"), "spin_rov_say yok"
    assert hasattr(at, "spin_m2"), "spin_m2 yok"
    assert hasattr(at, "chk_sessiz"), "chk_sessiz yok"
    _pass("AlanTaramaSekmesi: tüm widget'lar mevcut")
except Exception as e:
    _fail("AlanTaramaSekmesi widget'ları", e)

# AramaKurtarmaSekmesi
try:
    ak = AramaKurtarmaSekmesi(None)
    assert hasattr(ak, "spin_grup"), "spin_grup yok"
    assert hasattr(ak, "spin_guven"), "spin_guven yok"
    assert hasattr(ak, "txt_model"), "txt_model yok"
    assert hasattr(ak, "spin_rov_say"), "spin_rov_say yok"
    assert hasattr(ak, "txt_sinif"), "txt_sinif yok"
    _pass("AramaKurtarmaSekmesi: tüm widget'lar mevcut")
except Exception as e:
    _fail("AramaKurtarmaSekmesi widget'ları", e)

# ImhaSekmesi
try:
    im = ImhaSekmesi(None)
    assert hasattr(im, "cmb_mod"), "cmb_mod yok"
    assert hasattr(im, "spin_k_grup"), "spin_k_grup yok"
    assert hasattr(im, "spin_k_mesafe"), "spin_k_mesafe yok"
    assert hasattr(im, "spin_a_grup"), "spin_a_grup yok"
    assert hasattr(im, "txt_a_sinif"), "txt_a_sinif yok"
    assert hasattr(im, "spin_a_mesafe"), "spin_a_mesafe yok"
    assert hasattr(im, "txt_a_model"), "txt_a_model yok"
    assert hasattr(im, "spin_a_rov_say"), "spin_a_rov_say yok"
    assert hasattr(im, "lbl_sonuc"), "lbl_sonuc yok"
    assert hasattr(im, "_durdur"), "_durdur metodu yok"
    assert hasattr(im, "_guncelle"), "_guncelle metodu yok"
    assert im.txt_a_model.text() == "yolov8n.pt", "txt_a_model varsayılan yanlış"
    assert im.spin_a_rov_say.minimum() == 0, "spin_a_rov_say min yanlış"
    _pass("ImhaSekmesi: tüm widget'lar + metodlar mevcut")
except Exception as e:
    _fail("ImhaSekmesi widget'ları", e)

# ImhaSekmesi mod geçişi (isHidden() kullan — widget show edilmemis olabilir)
try:
    im2 = ImhaSekmesi(None)
    assert not im2.w_koordinat.isHidden(), "Koordinat modu başlangıçta gizli olmamalı"
    assert im2.w_alan.isHidden(), "Alan modu başlangıçta gizli olmalı"
    im2.cmb_mod.setCurrentIndex(1)
    assert im2.w_koordinat.isHidden(), "Koordinat modu gizli olmalı"
    assert not im2.w_alan.isHidden(), "Alan modu görünür olmalı"
    im2.cmb_mod.setCurrentIndex(0)
    assert not im2.w_koordinat.isHidden(), "Koordinat modu tekrar görünür olmalı"
    _pass("ImhaSekmesi mod geçişi (koordinat ↔ alan)")
except Exception as e:
    _fail("ImhaSekmesi mod geçişi", e)

# DurumSekmesi
try:
    ds = DurumSekmesi(None)
    ds._timer.stop()  # Test sırasında timer çalışmasın
    assert hasattr(ds, "tarayici"), "tarayici yok"
    assert hasattr(ds, "_yenile"), "_yenile metodu yok"
    assert hasattr(ds, "_tum_durdur"), "_tum_durdur metodu yok"
    _pass("DurumSekmesi: widget'lar + metodlar mevcut")
except Exception as e:
    _fail("DurumSekmesi widget'ları", e)

# DurumSekmesi _yenile imports aktif_gorevler_bilgisi
try:
    import inspect
    src = inspect.getsource(DurumSekmesi._yenile)
    assert "aktif_gorevler_bilgisi" in src, "_yenile aktif_gorevler_bilgisi import etmiyor"
    assert "AKTİF GÖREVLER" in src, "_yenile AKTİF GÖREVLER bölümü yok"
    _pass("DurumSekmesi._yenile: aktif görev tablosu var")
except Exception as e:
    _fail("DurumSekmesi._yenile içerik kontrolü", e)

# GorevPanel sekme sayısı
try:
    gp = GorevPanel(None)
    assert gp.sekme.count() == 5, f"Sekme sayısı {gp.sekme.count()} (5 olmalı)"
    tabs = [gp.sekme.tabText(i) for i in range(gp.sekme.count())]
    assert any("Durum" in t for t in tabs), "Durum sekmesi yok"
    assert any("Hareket" in t for t in tabs), "Hareket sekmesi yok"
    assert any("Alan" in t or "Tarama" in t for t in tabs), "Alan Tarama sekmesi yok"
    assert any("Arama" in t for t in tabs), "Arama Kurtarma sekmesi yok"
    assert any("İmha" in t for t in tabs), "İmha sekmesi yok"
    _pass(f"GorevPanel: {gp.sekme.count()} sekme, tüm isimler doğru")
except Exception as e:
    _fail("GorevPanel sekme sayısı/isimleri", e)


# ═══════════════════════════════════════════════════════════════════════════════
# BÖLÜM 3: SurucuPanel — race condition + TTL token
# ═══════════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 60)
print("TEST 3: SurucuPanel race condition + TTL token")
print("=" * 60)

try:
    from UI.paneller.surucu_panel import SurucuPanel, LiderGrubu
    _pass("surucu_panel imports")
except ImportError as e:
    _fail("surucu_panel imports", e)
    sys.exit(1)

# _bekleyen_hareket tip kontrolü
try:
    sp = SurucuPanel(None)
    assert isinstance(sp._bekleyen_hareket, dict), "_bekleyen_hareket dict olmalı"
    _pass("SurucuPanel._bekleyen_hareket dict tipi")
except Exception as e:
    _fail("SurucuPanel._bekleyen_hareket tipi", e)

# _lider_olustur TTL token
try:
    sp2 = SurucuPanel(None)
    _veri = {"id": 0, "rol": 0, "grup_id": 0, "batarya": 1.0,
             "hiz": 0.0, "gps": (0, 0, 0), "gat_kodu": 0, "gorev": "idle"}
    sp2._veri = {0: _veri}
    sp2._base.add(0)
    sp2._us.rov_ekle(0, _veri)
    sp2._son_sim_state[0] = (0, 0)
    sp2._init_ok = True  # İlk bağlantı zaten yapıldı

    sp2._lider_olustur(0, emit_komut=True)
    assert 0 in sp2._liderler, "ROV-0 lider olmalı"
    assert 0 in sp2._bekleyen_hareket, "Token set edilmeli"
    expire = sp2._bekleyen_hareket[0]
    assert expire > time.monotonic(), "Token gelecekte bitmeli"
    _pass("_lider_olustur: ROV lider + TTL token set")
except Exception as e:
    _fail("_lider_olustur TTL token", e)

# Token, aynı state update'de korunmalı
try:
    sp2.rov_listesini_guncelle([_veri])  # Sim hâlâ eski state
    assert 0 in sp2._bekleyen_hareket, "Token eski-state update'de silinmemeli"
    assert 0 in sp2._liderler, "Lider eski-state update'de kaldırılmamalı"
    _pass("Token survival: eski_state update'de korunuyor")
except Exception as e:
    _fail("Token survival eski-state", e)

# Sim onayından sonra token tüketilmeli
try:
    yeni_veri = {**_veri, "rol": 1, "grup_id": 1}
    sp2.rov_listesini_guncelle([yeni_veri])
    assert 0 not in sp2._bekleyen_hareket, "Token sim onayından sonra tüketilmeli"
    assert 0 in sp2._liderler, "Lider sim onayından sonra da lider olmalı"
    _pass("Token tüketimi: sim onayı sonrası doğru")
except Exception as e:
    _fail("Token tüketimi sim onayı", e)

# TTL süresi dolan token temizlenmeli
try:
    sp3 = SurucuPanel(None)
    sp3._bekleyen_hareket[99] = time.monotonic() - 1.0  # Süresi dolmuş
    sp3._bekleyen_hareket[100] = time.monotonic() + 30.0  # Geçerli
    sp3._init_ok = True
    sp3.rov_listesini_guncelle([])  # Boş update — temizlik tetiklenmeli
    assert 99 not in sp3._bekleyen_hareket, "Süresi dolmuş token temizlenmeli"
    assert 100 in sp3._bekleyen_hareket, "Geçerli token korunmalı"
    _pass("TTL token temizleme: süresi dolanlar siliniyor")
except Exception as e:
    _fail("TTL token temizleme", e)

# _gorev_durdur_hepsi metodu
try:
    _lider_veri = {"id": 0, "batarya": 1.0, "hiz": 0.0, "gps": (0, 0, 0),
                   "gat_kodu": 0, "gorev": "idle", "rol": 1, "grup_id": 1}
    lg = LiderGrubu(0, _lider_veri, 1, None)
    assert hasattr(lg, "_gorev_durdur_hepsi"), "_gorev_durdur_hepsi metodu yok"
    assert hasattr(lg, "_gorev_baslat"), "_gorev_baslat metodu yok"
    assert hasattr(lg, "_gorev_durdur"), "_gorev_durdur metodu yok"
    _pass("LiderGrubu: görev metodları mevcut")
except Exception as e:
    _fail("LiderGrubu görev metodları", e)

# _gorev_baslat inspect (tüm görevleri durdurma komutu içermeli)
try:
    import inspect
    src = inspect.getsource(LiderGrubu._gorev_baslat)
    assert "_gorev_durdur_hepsi" in src, "_gorev_baslat tüm görevleri durdurmuyor"
    assert "sessiz=True" in src, "_gorev_baslat sessiz=True kullanmıyor"
    assert "_rov_hedefleri" in src or "temiz" in src.lower(), \
        "_gorev_baslat nav hedeflerini temizlemiyor"
    _pass("_gorev_baslat: durdur+temizle+başlat deseni doğru")
except Exception as e:
    _fail("_gorev_baslat içerik kontrolü", e)

# _gorev_durdur_hepsi inspect (tüm tipler var mı)
try:
    src_d = inspect.getsource(LiderGrubu._gorev_durdur_hepsi)
    assert "alan_tarama_gorevi" in src_d, "alan_tarama durdurulmuyor"
    assert "arama_kurtarma_gorevi" in src_d, "arama_kurtarma durdurulmuyor"
    assert "imha_gorevi" in src_d, "imha durdurulmuyor"
    _pass("_gorev_durdur_hepsi: 3 görev tipi de durdurulmuyor")
except Exception as e:
    _fail("_gorev_durdur_hepsi içerik kontrolü", e)


# ═══════════════════════════════════════════════════════════════════════════════
# BÖLÜM 4: Komut string doğruluğu
# ═══════════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 60)
print("TEST 4: Komut string doğruluğu")
print("=" * 60)

# Alan tarama komutu
try:
    from UI.paneller.gorev_panel import AlanTaramaSekmesi
    at2 = AlanTaramaSekmesi(None)
    at2.spin_grup.setValue(2)
    at2.spin_derinlik.setValue(-30.0)
    at2.spin_serit.setValue(20.0)
    at2.spin_m2.setValue(500.0)
    at2.spin_rov_say.setValue(0)  # 0 = hepsi (gereken_rov_sayisi parametresi eklenmemeli)
    at2.chk_sessiz.setChecked(True)

    # _basla'nın ürettiği komutu yakalamak için signal'ı bağla
    captured = []
    at2.komut_uretildi.connect(lambda k, a: captured.append(k))
    at2._basla()
    assert captured, "_basla komut üretmeli"
    k = captured[-1]
    assert "grup_id=2" in k, f"grup_id yanlış: {k}"
    assert "derinlik=-30.0" in k, f"derinlik yanlış: {k}"
    assert "serit_araligi=20.0" in k, f"serit_araligi yanlış: {k}"
    assert "rov_basina_varsayilan_alan_m2=500.0" in k, f"m2 parametresi yok: {k}"
    assert "gereken_rov_sayisi" not in k, f"gereken_rov_sayisi 0 iken olmamalı: {k}"
    assert "sessiz=True" in k, f"sessiz=True yok: {k}"
    _pass("AlanTaramaSekmesi komut string doğru")
except Exception as e:
    _fail("AlanTaramaSekmesi komut string", e)

# gereken_rov_sayisi n>0 ise eklenmeli
try:
    at3 = AlanTaramaSekmesi(None)
    at3.spin_rov_say.setValue(3)
    captured2 = []
    at3.komut_uretildi.connect(lambda k, a: captured2.append(k))
    at3._basla()
    k2 = captured2[-1]
    assert "gereken_rov_sayisi=3" in k2, f"gereken_rov_sayisi=3 yok: {k2}"
    _pass("AlanTaramaSekmesi: gereken_rov_sayisi n>0 ekleniyor")
except Exception as e:
    _fail("AlanTaramaSekmesi gereken_rov_sayisi n>0", e)

# Arama kurtarma komutu
try:
    from UI.paneller.gorev_panel import AramaKurtarmaSekmesi
    ak2 = AramaKurtarmaSekmesi(None)
    ak2.spin_grup.setValue(1)
    ak2.txt_sinif.setText("person, diver")
    ak2.spin_guven.setValue(0.6)
    captured3 = []
    ak2.komut_uretildi.connect(lambda k, a: captured3.append(k))
    ak2._basla()
    k3 = captured3[-1]
    assert "sessiz=True" in k3, f"sessiz=True yok: {k3}"
    assert "min_confidence=0.6" in k3, f"min_confidence yok: {k3}"
    assert "'person'" in k3 or "person" in k3, f"hedef sınıf yok: {k3}"
    _pass("AramaKurtarmaSekmesi komut string doğru")
except Exception as e:
    _fail("AramaKurtarmaSekmesi komut string", e)

# İmha koordinat komutu
try:
    from UI.paneller.gorev_panel import ImhaSekmesi
    im3 = ImhaSekmesi(None)
    im3.cmb_mod.setCurrentIndex(0)  # Koordinat
    im3.spin_k_grup.setValue(1)
    im3.spin_hx.setValue(50.0)
    im3.spin_hy.setValue(60.0)
    im3.spin_hz.setValue(-10.0)
    im3.spin_k_mesafe.setValue(5.0)
    captured4 = []
    im3.komut_uretildi.connect(lambda k, a: captured4.append(k))
    im3._basla()
    k4 = captured4[-1]
    assert "koordinat_imha_baslat" in k4, f"koordinat_imha_baslat yok: {k4}"
    assert "grup_id=1" in k4, f"grup_id yok: {k4}"
    assert "sessiz=True" in k4, f"sessiz=True yok: {k4}"
    _pass("ImhaSekmesi koordinat komut string doğru")
except Exception as e:
    _fail("ImhaSekmesi koordinat komut string", e)

# İmha alan komutu
try:
    im4 = ImhaSekmesi(None)
    im4.cmb_mod.setCurrentIndex(1)  # Alan
    im4.spin_a_grup.setValue(2)
    im4.txt_a_sinif.setText("mine")
    im4.txt_a_model.setText("custom.pt")
    im4.spin_a_rov_say.setValue(2)
    captured5 = []
    im4.komut_uretildi.connect(lambda k, a: captured5.append(k))
    im4._basla()
    k5 = captured5[-1]
    assert "alan_imha_baslat" in k5, f"alan_imha_baslat yok: {k5}"
    assert "model_path='custom.pt'" in k5, f"model_path yok: {k5}"
    assert "gereken_rov_sayisi=2" in k5, f"gereken_rov_sayisi yok: {k5}"
    assert "sessiz=True" in k5, f"sessiz=True yok: {k5}"
    _pass("ImhaSekmesi alan komut string doğru")
except Exception as e:
    _fail("ImhaSekmesi alan komut string", e)


# ═══════════════════════════════════════════════════════════════════════════════
# BÖLÜM 5: SurucuPanel grup davranışı
# ═══════════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 60)
print("TEST 5: SurucuPanel grup davranışı")
print("=" * 60)

def _make_rov(rid, rol=0, gid=0):
    return {"id": rid, "rol": rol, "grup_id": gid, "batarya": 1.0,
            "hiz": 0.0, "gps": (rid * 10, 0, 0), "gat_kodu": 0, "gorev": "idle"}

# _ilk_yerles: liderli sim state
try:
    sp4 = SurucuPanel(None)
    rovlar = [_make_rov(0, rol=1, gid=1), _make_rov(1, rol=0, gid=1), _make_rov(2)]
    sp4.rov_listesini_guncelle(rovlar)  # _init_ok=False → _ilk_yerles çağrılır
    assert 0 in sp4._liderler, "ROV-0 lider olmalı"
    assert 1 in sp4._liderler[0].takipci_idleri(), "ROV-1 takipçi olmalı"
    assert 2 in sp4._base, "ROV-2 üste olmalı"
    _pass("_ilk_yerles: lider+takipçi+üs doğru yerleşti")
except Exception as e:
    _fail("_ilk_yerles grup yerleşimi", e)

# Harici sim değişikliği: ROV-2 gruba katıldı
try:
    sp4._init_ok = True  # İlk bağlantı bitti
    yeni_rovlar = [_make_rov(0, rol=1, gid=1), _make_rov(1, rol=0, gid=1), _make_rov(2, rol=0, gid=1)]
    sp4.rov_listesini_guncelle(yeni_rovlar)
    assert 2 in sp4._liderler[0].takipci_idleri(), "ROV-2 gruba katılmalıydı"
    assert 2 not in sp4._base, "ROV-2 üste olmamalı artık"
    _pass("Harici sim değişikliği: ROV-2 gruba katılması yansıdı")
except Exception as e:
    _fail("Harici sim değişikliği ROV-2", e)

# _tum_use_al: tüm ROV'lar üsse dönmeli (widget seviyesinde)
try:
    sp5 = SurucuPanel(None)
    rovlar5 = [_make_rov(0, rol=1, gid=1), _make_rov(1, rol=0, gid=1)]
    sp5.rov_listesini_guncelle(rovlar5)
    assert 0 in sp5._liderler
    komutlar = []
    sp5.komut_uretildi.connect(lambda k, a: komutlar.append(k))
    sp5._tum_use_al()
    # Liderler ve takipçiler üsse alınmalı → liderler listesi boşalmalı
    assert len(sp5._liderler) == 0, f"Liderler temizlenmeli: {list(sp5._liderler.keys())}"
    _pass("_tum_use_al: tüm gruplar çözüldü")
except Exception as e:
    _fail("_tum_use_al", e)


# ═══════════════════════════════════════════════════════════════════════════════
# ÖZET
# ═══════════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 60)
print("UI TEST SONUÇLARI")
print("=" * 60)
total   = sum(len(v) for v in _results.values())
passed  = len(_results["passed"])
failed  = len(_results["failed"])
skipped = len(_results["skipped"])
print(f"Toplam: {total}  ✅ {passed}  ❌ {failed}  ⏭  {skipped}")

if _results["failed"]:
    print("\nBaşarısız:")
    for name, err in _results["failed"]:
        print(f"  ❌ {name}: {err}")

if __name__ == "__main__":
    sys.exit(1 if failed else 0)
