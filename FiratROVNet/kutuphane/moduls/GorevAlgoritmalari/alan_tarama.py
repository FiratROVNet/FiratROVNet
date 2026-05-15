from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .ortak import (
    TaramaAlani,
    alan_normalize,
    aktif_grup_rovlari,
    kopma_menzili,
    lider_rov_bul,
    rov_gorev_ata,
    rov_gorev_bosalt,
)
from ..rov_deger_onerisi import GorevHedefi
from ....config import AlanTaramaAyarlari


@dataclass
class AlanTaramaPlani:
    grup_id: int
    alan: TaramaAlani
    lider_id: int | None
    rota_by_rov: dict[int, list[tuple[float, float]]] = field(default_factory=dict)
    derinlik: float = field(default=AlanTaramaAyarlari.DERINLIK)
    serit_araligi: float = field(default=AlanTaramaAyarlari.SERIT_ARALIGI)
    iletisim_menzili: float = field(default=AlanTaramaAyarlari.ILETISIM_MENZILI)
    gorev_adi: str = field(default=AlanTaramaAyarlari.GOREV_ADI)
    kaynak_grup_id: int | None = None
    yaklasma_hedefi: tuple[float, float, float] | None = None
    asama: str = field(default=AlanTaramaAyarlari.ASAMA)
    # Lider taramanın başladığı konum — görev tamamlanınca geri dönülür
    baslangic_konum: tuple[float, float] | None = None
    # True → her ROV kendi şeridini bağımsız tarar (arama_kurtarma modu)
    herkese_rota: bool = False
    # True → rota bitince durmadan baştan tekrar başlat (sonsuz devriye)
    surekli_tarama: bool = False


class AlanTaramaGorevi:
    """
    Boustrophedon/lawnmower alan tarama gorevi.

    Grup ROV'larini paralel hatlara dizer. Lider merkez hatta kalir; diger ROV'lar
    liderden kopma menzili icinde kalan offsetlerde tarama yapar. Alan genis ise
    ayni formasyon birden fazla bantta gezdirilir.
    """

    def __init__(self, filo_ref):
        self.filo = filo_ref
        self.aktif_planlar: dict[int, AlanTaramaPlani] = {}

    def plan_olustur(
        self,
        grup_id: int,
        alan,
        derinlik: float = -20.0,
        serit_araligi: float = 15.0,
        rov_basina_varsayilan_alan_m2: float = 400.0,
        iletisim_menzili: float | None = None,
        gereken_rov_sayisi: int | None = None,
        gorev_adi: str = "alan_tarama",
    ) -> AlanTaramaPlani:
        tarama_alani = alan_normalize(alan, z=derinlik)

        # filo.g_rovs grup_id=0'ı otomatik olarak 1'e taşır (numaralandırma 1'den başlar).
        # Kullanıcı grup_id=0 verirse mevcut ilk gruba yönlendir.
        if grup_id == 0 and not aktif_grup_rovlari(self.filo, 0):
            mevcut = sorted(k for k in self.filo.g_rovs.keys() if k > 0)
            if mevcut:
                grup_id = mevcut[0]

        # Grup rovlarını doğrudan al — lider (role=1) dahil.
        # en_iyi_rovlari_sec rol_carpani=0.0 ile lideri devre dışı bırakıyordu;
        # gorev_grubu_olustur group_id'leri değiştirip UI grubunu bozuyordu.
        rovs: list[Any] = aktif_grup_rovlari(self.filo, grup_id)
        if gereken_rov_sayisi is not None and int(gereken_rov_sayisi) > 0:
            rovs = rovs[:int(gereken_rov_sayisi)]
        if not rovs:
            mevcut_gruplar = sorted(k for k in self.filo.g_rovs.keys() if k > 0)
            raise ValueError(
                f"Grup-{grup_id} için ROV bulunamadı. "
                f"Mevcut gruplar: {mevcut_gruplar or 'yok — ROVları önce gruba atayın.'}"
            )

        kaynak_grup_id = grup_id

        lider = lider_rov_bul(self.filo, grup_id, rovs)
        lider_id = int(getattr(lider, "id", rovs[0].id)) if lider else int(rovs[0].id)
        rovs = sorted(rovs, key=lambda r: (0 if getattr(r, "id", None) == lider_id else 1, int(getattr(r, "id", 0))))

        menzil = kopma_menzili(iletisim_menzili)
        min_swath = max(1.0, float(rov_basina_varsayilan_alan_m2) ** 0.5)
        swath = max(1.0, min(float(serit_araligi), min_swath, menzil * 0.45))
        offsets = self._offsetler(len(rovs), swath, menzil)
        band_genisligi = max(swath, (max(offsets) - min(offsets)) + swath)

        band_merkezleri = []
        x = tarama_alani.x_min + band_genisligi * 0.5
        while x <= tarama_alani.x_max + 1e-6:
            band_merkezleri.append(min(max(x, tarama_alani.x_min), tarama_alani.x_max))
            x += band_genisligi
        if not band_merkezleri:
            band_merkezleri = [tarama_alani.center[0]]

        rota_by_rov: dict[int, list[tuple[float, float]]] = {int(r.id): [] for r in rovs}
        ters = False
        for band_x in band_merkezleri:
            for rov, offset in zip(rovs, offsets):
                rx = min(max(band_x + offset, tarama_alani.x_min), tarama_alani.x_max)
                if ters:
                    rota_by_rov[int(rov.id)].extend([(rx, tarama_alani.y_max), (rx, tarama_alani.y_min)])
                else:
                    rota_by_rov[int(rov.id)].extend([(rx, tarama_alani.y_min), (rx, tarama_alani.y_max)])
            ters = not ters

        return AlanTaramaPlani(
            grup_id=grup_id,
            alan=tarama_alani,
            lider_id=lider_id,
            rota_by_rov=rota_by_rov,
            derinlik=float(derinlik),
            serit_araligi=swath,
            iletisim_menzili=menzil,
            gorev_adi=gorev_adi,
            kaynak_grup_id=kaynak_grup_id,
            yaklasma_hedefi=tarama_alani.center,
        )

    def baslat(
        self,
        grup_id: int,
        alan,
        derinlik: float = -20.0,
        serit_araligi: float = 15.0,
        rov_basina_varsayilan_alan_m2: float = 400.0,
        iletisim_menzili: float | None = None,
        gereken_rov_sayisi: int | None = None,
        gorev_adi: str = "alan_tarama",
        bagimsiz_mod: bool = True,
        herkese_rota: bool = False,
        surekli_tarama: bool = False,
        sessiz: bool = True,
    ) -> AlanTaramaPlani:
        plan = self.plan_olustur(
            grup_id=grup_id,
            alan=alan,
            derinlik=derinlik,
            serit_araligi=serit_araligi,
            rov_basina_varsayilan_alan_m2=rov_basina_varsayilan_alan_m2,
            iletisim_menzili=iletisim_menzili,
            gereken_rov_sayisi=gereken_rov_sayisi,
            gorev_adi=gorev_adi,
        )
        plan.herkese_rota = herkese_rota
        plan.surekli_tarama = surekli_tarama
        self.aktif_planlar[grup_id] = plan
        self._yaklasma_baslat(plan, sessiz=sessiz)
        return plan

    def durdur(self, grup_id: int, lideri_takip_et: bool = True) -> None:
        plan = self.aktif_planlar.pop(grup_id, None)
        if not plan:
            return
        try:
            self.filo.aktif_formasyon[plan.grup_id] = None
        except Exception:
            pass
        for rov_id in plan.rota_by_rov:
            rov_gorev_bosalt(self.filo, rov_id, lideri_takip_et=lideri_takip_et)

    def guncelle(self, grup_id: int | None = None, lideri_takip_et: bool = True) -> list[int]:
        """Rotasi biten plan ROV'larini idle'a alir. Donus: biten grup idleri."""
        biten_gruplar = []
        planlar = list(self.aktif_planlar.items())
        for p_grup_id, plan in planlar:
            if grup_id is not None and p_grup_id != grup_id:
                continue
            tamamlandi = True
            if plan.asama == "yaklasma":
                self._plan_liderini_sabitle(plan)
                if self._yaklasma_tamamlandi_mi(plan):
                    self._tarama_baslat(plan, lideri_bagimsiz_yap=True, sessiz=False)
                continue
            # herkese_rota=True ise dışarıdan (AramaKurtarmaGorevi) yönetilir — burada bitirme
            if plan.herkese_rota:
                continue
            # Tarama bitimi: sadece lider rotasını tamamladı mı? Takipçiler formasyonda zaten.
            kontrol_rov = plan.lider_id if plan.lider_id is not None else next(iter(plan.rota_by_rov), None)
            if kontrol_rov is None or not self._rov_tarama_bitti_mi(kontrol_rov):
                tamamlandi = False
            if tamamlandi:
                if plan.surekli_tarama:
                    # Sonsuz devriye: rota bitince baştan tekrar başlat, durdurma
                    self._tarama_baslat(plan, lideri_bagimsiz_yap=False, sessiz=True)
                else:
                    self.durdur(p_grup_id, lideri_takip_et=lideri_takip_et)
                    biten_gruplar.append(p_grup_id)
        return biten_gruplar

    def _yaklasma_baslat(self, plan: AlanTaramaPlani, sessiz: bool) -> None:
        # Liderin mevcut konumunu başlangıç noktası olarak kaydet (görev bitince geri döner)
        if plan.lider_id is not None and plan.baslangic_konum is None:
            try:
                gps = self.filo.get(plan.lider_id, "gps")
                plan.baslangic_konum = (float(gps[0]), float(gps[1]))
            except Exception:
                plan.baslangic_konum = None

        hedef_bilgisi = GorevHedefi(gorev_adi=plan.gorev_adi, grup_id=plan.grup_id, alan=plan.alan)
        for rov_id in plan.rota_by_rov:
            rov = self.filo.find_rov_by_id(rov_id)
            if not rov:
                continue
            eski_gorev = str(getattr(getattr(rov, "gnc", None), "gorev", "idle") or "idle")
            if eski_gorev != "idle" and eski_gorev != plan.gorev_adi:
                rov_gorev_bosalt(self.filo, rov_id, lideri_takip_et=False)
            # Lider bağımsız hareket eder (mod=0), takipçiler formasyonu takip eder (mod=1)
            lider_mi = (rov_id == plan.lider_id)
            rov_gorev_ata(rov, plan.gorev_adi, mod=(0 if lider_mi else 1), gorev_hedef=hedef_bilgisi)

        # formasyon_sec(dinamik=True) ÇAĞRILMIYOR — arka plan thread'inden çağrılırsa
        # done_event.wait(10s) ile bloklar → Ursina frame loop durur → "bağlantı yok".
        # Bunun yerine aktif_formasyon[g_id]'yi doğrudan set ediyoruz (ağır hull hesabı yok).
        # Mevcut formasyon varsa koru; yoksa varsayılan LINE ata → takipçiler mod=1 ile çalışır.
        try:
            af = getattr(self.filo, "aktif_formasyon", None)
            if not isinstance(af, dict):
                self.filo.aktif_formasyon = {}
                af = self.filo.aktif_formasyon
            if not af.get(plan.grup_id):
                af[plan.grup_id] = {
                    "id": "LINE", "aralik": 10, "is_3d": False,
                    "yaw": 0, "g_id": plan.grup_id,
                }
        except Exception:
            pass

        if plan.lider_id is not None and plan.yaklasma_hedefi is not None:
            # _plan_liderini_sabitle burada kasıtlı olarak ÇAĞRILMIYOR.
            # Çağrılırsa rov.role değişir → UI _sim_statine_gore_yerlestir tetikler
            # → _lider_kaldir tüm grubu yok eder → "siliyor sonra yüklüyor" görünür.
            # guncelle() zaten her frame yaklasma aşamasında _plan_liderini_sabitle
            # çağırır; roller 1 tick içinde kendiliğinden düzelir.
            self.filo.git_path(plan.lider_id, plan.yaklasma_hedefi, ai=True, isaret=True)
        plan.asama = "yaklasma"

    def _yaklasma_tamamlandi_mi(self, plan: AlanTaramaPlani) -> bool:
        # Sadece liderin yaklaşma noktasına varması yeterli — takipçi formasyon
        # bekleme koşulu taramayı süresiz blokluyor.
        if plan.lider_id is None:
            return True
        lider = self.filo.find_rov_by_id(plan.lider_id)
        if lider is None or plan.yaklasma_hedefi is None:
            return self._rov_tarama_bitti_mi(plan.lider_id)
        try:
            gps = self.filo.get(plan.lider_id, "gps")
            return self._lider_yaklasma_hedefine_vardi(plan, gps)
        except Exception:
            return self._rov_tarama_bitti_mi(plan.lider_id)

    def _lider_yaklasma_hedefine_vardi(self, plan: AlanTaramaPlani, gps) -> bool:
        rota = getattr(self.filo, "_git_nokta_listesi", {}).get(plan.lider_id) if plan.lider_id is not None else None
        indeks = getattr(self.filo, "_git_mevcut_nokta_indeksi", {}).get(plan.lider_id, 0) if plan.lider_id is not None else 0
        tolerans = max(10.0, min(25.0, plan.iletisim_menzili * 0.35))

        if rota:
            try:
                son_nokta = rota[-1]
                sdx = float(gps[0]) - float(son_nokta[0])
                sdy = float(gps[1]) - float(son_nokta[1])
                if (sdx * sdx + sdy * sdy) ** 0.5 <= tolerans:
                    return True
                hedef_nokta = son_nokta if indeks >= len(rota) - 1 else rota[int(indeks)]
                dx = float(gps[0]) - float(hedef_nokta[0])
                dy = float(gps[1]) - float(hedef_nokta[1])
            except Exception:
                return False
            return indeks >= len(rota) - 1 and (dx * dx + dy * dy) ** 0.5 <= tolerans

        dx = float(gps[0]) - float(plan.yaklasma_hedefi[0])
        dy = float(gps[1]) - float(plan.yaklasma_hedefi[1])
        return (dx * dx + dy * dy) ** 0.5 <= tolerans

    def _takipciler_formasyona_yakin_mi(self, plan: AlanTaramaPlani) -> bool:
        hedefler = getattr(self.filo, "_formasyon_hedefleri", {}) or {}
        tolerans = plan.iletisim_menzili
        lider_gps = None
        if plan.lider_id is not None:
            try:
                lider_gps = self.filo.get(plan.lider_id, "gps")
            except Exception:
                lider_gps = None
        lider_takip_toleransi = max(10.0, min(plan.iletisim_menzili * 0.8, 35.0))
        for rov_id in plan.rota_by_rov:
            if rov_id == plan.lider_id:
                continue
            rov = self.filo.find_rov_by_id(rov_id)
            if rov is None:
                continue
            hedef = hedefler.get(rov_id)
            if not hedef:
                if lider_gps is None:
                    return False
                try:
                    gps = self.filo.get(rov_id, "gps")
                    dx = float(gps[0]) - float(lider_gps[0])
                    dy = float(gps[1]) - float(lider_gps[1])
                except Exception:
                    return False
                if (dx * dx + dy * dy) ** 0.5 > lider_takip_toleransi:
                    return False
                continue
            try:
                hedef_pos = hedef.get("pozisyon") if isinstance(hedef, dict) else hedef
                gps = self.filo.get(rov_id, "gps")
                dx = float(gps[0]) - float(hedef_pos[0])
                dy = float(gps[1]) - float(hedef_pos[1])
            except Exception:
                return False
            formasyon_mesafesi = (dx * dx + dy * dy) ** 0.5
            if formasyon_mesafesi <= tolerans:
                continue
            if lider_gps is None:
                return False
            try:
                ldx = float(gps[0]) - float(lider_gps[0])
                ldy = float(gps[1]) - float(lider_gps[1])
            except Exception:
                return False
            if (ldx * ldx + ldy * ldy) ** 0.5 > lider_takip_toleransi:
                return False
        return True

    def _rov_tarama_bitti_mi(self, rov_id: int) -> bool:
        if bool(getattr(self.filo, "_git_nokta_listesi", {}).get(rov_id)):
            return False
        if bool(getattr(self.filo, "_git_mevcut_nokta_indeksi", {}).get(rov_id)):
            return False
        return True

    def _tarama_baslat(self, plan: AlanTaramaPlani, lideri_bagimsiz_yap: bool, sessiz: bool) -> None:
        # Grup birlikte tarar: lider rotayı alır (mod=0), takipçiler formasyonu korur (mod=1).
        # Formasyon silinmez — takipçiler lideri formasyonda takip eder, grup dağılmaz.
        self._plan_liderini_sabitle(plan)
        if not sessiz:
            print(f"✅ [ALAN_TARAMA] Grup-{plan.grup_id} yaklaşma tamamlandı, grup taraması başlatılıyor.")
        hedef_bilgisi = GorevHedefi(gorev_adi=plan.gorev_adi, grup_id=plan.grup_id, alan=plan.alan)

        # ── Herkese rota modu (arama_kurtarma): tüm ROVlar bağımsız şeritlerini A* ile tarar ──
        if plan.herkese_rota:
            # Formasyonu kapat — herkes bağımsız hareket edecek
            try:
                af = getattr(self.filo, "aktif_formasyon", None)
                if isinstance(af, dict):
                    af[plan.grup_id] = None
            except Exception:
                pass
            for rov_id in plan.rota_by_rov:
                rov = self.filo.find_rov_by_id(rov_id)
                if not rov:
                    continue
                rov_gorev_ata(rov, plan.gorev_adi, mod=0, gorev_hedef=hedef_bilgisi)
                rov_rota = plan.rota_by_rov.get(rov_id)
                if not rov_rota:
                    continue
                try:
                    gps = self.filo.get(rov_id, "gps")
                    bslngc: tuple[float, float] = (float(gps[0]), float(gps[1]))
                except Exception:
                    bslngc = (float(rov_rota[0][0]), float(rov_rota[0][1]))
                genisletilmis = self._a_star_rota_genislet(bslngc, rov_rota, plan.derinlik)
                self.filo.git(rov_id, genisletilmis, z=plan.derinlik, ai=True, sessiz=sessiz)
            plan.asama = "tarama"
            return

        lider_rota = plan.rota_by_rov.get(plan.lider_id) if plan.lider_id is not None else None
        for rov_id in plan.rota_by_rov:
            rov = self.filo.find_rov_by_id(rov_id)
            if not rov:
                continue
            lider_mi = (rov_id == plan.lider_id)
            if lider_mi and lider_rota:
                # Lider: bağımsız mod, A* ile engel-kaçınmalı genişletilmiş rota
                rov_gorev_ata(rov, plan.gorev_adi, mod=0, gorev_hedef=hedef_bilgisi)

                # Liderin mevcut konumunu al (yaklaşma sonrası)
                try:
                    gps = self.filo.get(rov_id, "gps")
                    baslangic_2d: tuple[float, float] = (float(gps[0]), float(gps[1]))
                except Exception:
                    baslangic_2d = (
                        float(plan.yaklasma_hedefi[0]) if plan.yaklasma_hedefi else 0.0,
                        float(plan.yaklasma_hedefi[1]) if plan.yaklasma_hedefi else 0.0,
                    )

                # A* ile engel-kaçınmalı rota genişlet
                if not sessiz:
                    print(f"🔍 [ALAN_TARAMA] A* rota genişletiliyor ({len(lider_rota)} waypoint)…")
                genisletilmis = self._a_star_rota_genislet(baslangic_2d, lider_rota, plan.derinlik)
                if not sessiz:
                    print(f"✅ [ALAN_TARAMA] Genişletilmiş rota: {len(genisletilmis)} waypoint")

                # Görev tamamlanınca başlangıç noktasına dön
                if plan.baslangic_konum is not None:
                    son_nokta = genisletilmis[-1] if genisletilmis else baslangic_2d
                    ev_rota = self._a_star_rota_genislet(
                        son_nokta, [plan.baslangic_konum], plan.derinlik
                    )
                    genisletilmis.extend(ev_rota)
                    if not sessiz:
                        print(f"🏠 [ALAN_TARAMA] Eve dönüş yolu eklendi → {plan.baslangic_konum}")

                self.filo.git(rov_id, genisletilmis, z=plan.derinlik, ai=True, sessiz=sessiz)
            else:
                # Takipçi: formasyon modu (mod=1), lider peşinde kalır — grup dağılmaz
                rov_gorev_ata(rov, plan.gorev_adi, mod=1, gorev_hedef=hedef_bilgisi)
        # Lider git() çağrısı yapıldıktan sonra aşamayı değiştir.
        plan.asama = "tarama"

    def _plan_liderini_sabitle(self, plan: AlanTaramaPlani) -> None:
        if plan.lider_id is None:
            return
        for rov_id in plan.rota_by_rov:
            rov = self.filo.find_rov_by_id(rov_id)
            if rov is None:
                continue
            beklenen_rol = 1 if rov_id == plan.lider_id else 0
            # Zaten doğruysa atla — gereksiz rov.role değişimi UI cascade'i tetikler
            if int(getattr(rov, "role", 0)) == beklenen_rol:
                continue
            try:
                rov.set("rol", beklenen_rol)
            except Exception:
                rov.role = beklenen_rol
        leader_manager = getattr(self.filo, "leader_manager", None)
        if leader_manager is not None and hasattr(leader_manager, "mevcut_lider_id"):
            try:
                leader_manager.mevcut_lider_id[plan.grup_id] = int(plan.lider_id)
            except Exception:
                pass

    def _offsetler(self, rov_sayisi: int, swath: float, menzil: float) -> list[float]:
        if rov_sayisi <= 1:
            return [0.0]
        raw = [(i - (rov_sayisi - 1) * 0.5) * swath for i in range(rov_sayisi)]
        max_abs = max(abs(v) for v in raw)
        limit = max(1.0, menzil * 0.75)
        if max_abs <= limit:
            return raw
        scale = limit / max_abs
        return [v * scale for v in raw]

    def _a_star_rota_genislet(
        self,
        baslangic: tuple[float, float],
        rota: list[tuple[float, float]],
        derinlik: float,
    ) -> list[tuple[float, float]]:
        """
        Ham lawnmower rota waypoint'leri arasını A* ile engel-kaçınmalı
        alt-patikalarla doldurur. Dönen liste düzleştirilmiş (x, y) waypoint listesi.
        A* yoksa veya başarısız olursa orijinal rotayı düz döner.
        """
        helper = getattr(self.filo, "helper", None)
        if helper is None or not hasattr(helper, "_a_star_path_planla"):
            return list(rota)

        genisletilmis: list[tuple[float, float]] = []
        onceki = baslangic
        for hedef in rota:
            h = (float(hedef[0]), float(hedef[1]))
            try:
                alt_yol = helper._a_star_path_planla(onceki, h, duzlem_z=derinlik)
                if alt_yol and len(alt_yol) > 1:
                    # İlk nokta zaten önceki konumda — atla, fazla düğüm ekleme
                    genisletilmis.extend((float(p[0]), float(p[1])) for p in alt_yol[1:])
                else:
                    genisletilmis.append(h)
            except Exception:
                genisletilmis.append(h)
            onceki = h
        return genisletilmis
