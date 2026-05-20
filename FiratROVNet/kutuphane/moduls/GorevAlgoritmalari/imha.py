from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from ursina import Entity, color  # type: ignore[import]

from .arama_kurtarma import AramaKurtarmaGorevi, YoloTespit
from .ortak import mesafe_2d, rov_gorev_ata, rov_gorev_bosalt
from .ortak import gorev_grubu_olustur
from ..rov_deger_onerisi import GorevHedefi, en_iyi_rovlari_sec
from ....config import ImhaAyarlari


@dataclass(frozen=True)
class ImhaSonucu:
    basarili: bool
    rov_id: int | None = None
    hedef: tuple[float, float, float] | None = None
    tespit: YoloTespit | None = None
    mesaj: str = ""


class ImhaGorevi:
    """
    Koordinat veya YOLO tespiti ile imha gorevi.

    Koordinat verilirse en yakin ROV hedefe gider ve imha mesafesine girince
    entity_patlat cagrilir. Alan verilirse arama-kurtarma taramasi baslar;
    hedef sinif YOLO ile goruldugunde tespiti yapan ROV imha noktasina yonlendirilir.
    """

    def __init__(self, filo_ref):
        self.filo = filo_ref
        self.arama = AramaKurtarmaGorevi(filo_ref)
        self.hedef: tuple[float, float, float] | None = None
        self.gorevli_rov_id: int | None = None
        self.imha_mesafesi = ImhaAyarlari.IMHA_MESAFESI
        self.tespit: YoloTespit | None = None
        self.hedef_entity = None

    def koordinat_imha_baslat(
        self,
        grup_id: int,
        hedef: tuple[float, float, float],
        imha_mesafesi: float = ImhaAyarlari.IMHA_MESAFESI,
        sessiz: bool = True,
        hedef_entity=None,
    ) -> int:
        self.hedef = (float(hedef[0]), float(hedef[1]), float(hedef[2]))
        self.imha_mesafesi = float(imha_mesafesi)
        self.tespit = None
        self.hedef_entity = hedef_entity
        rov = self._en_yakin_rov(grup_id, self.hedef)
        if rov is None:
            raise ValueError(f"Grup-{grup_id} icinde imha icin aktif ROV yok.")
        gorev_grubu_olustur(self.filo, [rov])
        self.gorevli_rov_id = int(rov.id)
        eski_gorev = str(getattr(getattr(rov, "gnc", None), "gorev", "idle") or "idle")
        if eski_gorev != "idle" and eski_gorev != "imha":
            rov_gorev_bosalt(self.filo, self.gorevli_rov_id, lideri_takip_et=False)
        rov_gorev_ata(rov, "imha", mod=0, gorev_hedef=GorevHedefi(gorev_adi="imha", grup_id=grup_id, koordinat=self.hedef))
        self.filo.git(self.gorevli_rov_id, self.hedef[0], self.hedef[1], self.hedef[2], ai=True, sessiz=sessiz)
        return self.gorevli_rov_id

    def alan_imha_baslat(
        self,
        grup_id: int,
        alan,
        hedef_siniflari: Iterable[str] | None = None,
        model_path: str | None = None,
        derinlik: float = -20.0,
        imha_mesafesi: float = 8.0,
        gereken_rov_sayisi: int | None = None,
        sessiz: bool = True,
    ):
        self.imha_mesafesi = float(imha_mesafesi)
        self.hedef = None
        self.gorevli_rov_id = None
        self.tespit = None
        self.hedef_entity = None
        return self.arama.baslat(
            grup_id=grup_id,
            alan=alan,
            hedef_siniflari=hedef_siniflari,
            model_path=model_path,
            derinlik=derinlik,
            gereken_rov_sayisi=gereken_rov_sayisi,
            sessiz=sessiz,
        )

    def guncelle(self) -> ImhaSonucu | None:
        if self.hedef is None:
            tespit = self.arama.guncelle()
            if tespit is not None:
                self.tespit = tespit
                self.gorevli_rov_id = tespit.rov_id
                rov = self.filo.find_rov_by_id(tespit.rov_id)
                if rov is None:
                    return ImhaSonucu(False, tespit=tespit, mesaj="Tespit yapan ROV bulunamadi.")
                gps = self.filo.get(tespit.rov_id, "gps")
                z = float(gps[2]) if gps is not None and len(gps) >= 3 else -20.0
                # GPS sim koordinatlarını kullan (rov.x/rov.z Ursina koordinatı olduğundan karışıklık çıkar)
                self.hedef = (float(gps[0]), float(gps[1]), z)
                self.hedef_entity = None
                rov_gorev_ata(rov, "imha", mod=0, gorev_hedef=GorevHedefi(gorev_adi="imha", koordinat=self.hedef))
                self.filo.git(tespit.rov_id, self.hedef[0], self.hedef[1], self.hedef[2], ai=True, sessiz=True)
            else:
                return None

        if self.gorevli_rov_id is None or self.hedef is None:
            return None
        rov = self.filo.find_rov_by_id(self.gorevli_rov_id)
        if rov is None:
            return ImhaSonucu(False, hedef=self.hedef, tespit=self.tespit, mesaj="Gorevli ROV bulunamadi.")
        gps = self.filo.get(self.gorevli_rov_id, "gps")
        if gps is None:
            return None
        if mesafe_2d((float(gps[0]), float(gps[1])), (self.hedef[0], self.hedef[1])) > self.imha_mesafesi:
            return None

        self.filo.entity_patlat(self._hedef_entity_al(), parca_sayisi=80)
        sonuc = ImhaSonucu(True, rov_id=self.gorevli_rov_id, hedef=self.hedef, tespit=self.tespit, mesaj="Imha tamamlandi.")
        self.durdur(lideri_takip_et=True)
        return sonuc

    def durdur(self, lideri_takip_et: bool = True, gorselleri_koru: bool = False) -> None:
        self.arama.durdur(lideri_takip_et=lideri_takip_et, gorselleri_koru=gorselleri_koru)
        if self.gorevli_rov_id is not None:
            rov_gorev_bosalt(self.filo, self.gorevli_rov_id, lideri_takip_et=lideri_takip_et)
        self.hedef = None
        self.gorevli_rov_id = None
        self.tespit = None
        self.hedef_entity = None
        if not gorselleri_koru:
            from FiratROVNet.kutuphane.moduls.GorevAlgoritmalari.ortak import minimap_gorev_alanini_temizle
            minimap_gorev_alanini_temizle(self.filo)

    def _en_yakin_rov(self, grup_id: int, hedef: tuple[float, float, float]):
        adaylar = en_iyi_rovlari_sec(
            self.filo,
            GorevHedefi(gorev_adi="imha", grup_id=grup_id, koordinat=hedef),
            gereken_rov_sayisi=1,
        )
        if not adaylar:
            return None
        return adaylar[0]

    def _hedef_entity_al(self):
        if self.hedef_entity is not None:
            return self.hedef_entity
        if self.hedef is None:
            rov = self.filo.find_rov_by_id(self.gorevli_rov_id) if self.gorevli_rov_id is not None else None
            return rov
        self.hedef_entity = Entity(
            model="sphere",
            scale=1.5,
            position=(self.hedef[0], self.hedef[2], self.hedef[1]),
            color=color.rgba(255, 80, 40, 120),
            visible=False,
        )
        return self.hedef_entity
