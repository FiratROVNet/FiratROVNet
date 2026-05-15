from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .alan_tarama import AlanTaramaGorevi, AlanTaramaPlani
from ....config import AramaKurtarmaAyarlari


@dataclass(frozen=True)
class YoloTespit:
    rov_id: int
    class_name: str
    confidence: float
    bbox: tuple[float, float, float, float]


class AramaKurtarmaGorevi:
    """Alan tarama rotasini calistirir ve YOLO tespiti gorulunce gorevi tamamlar."""

    def __init__(self, filo_ref):
        self.filo = filo_ref
        self.alan_tarama = AlanTaramaGorevi(filo_ref)
        self.aktif_plan: AlanTaramaPlani | None = None
        self.hedef_siniflari: set[str] = set()
        self.min_confidence = AramaKurtarmaAyarlari.MIN_CONFIDENCE
        self.son_tespit: YoloTespit | None = None

    def baslat(
        self,
        grup_id: int,
        alan,
        hedef_siniflari: Iterable[str] | None = None,
        model_path: str = "yolov8n.pt",
        derinlik: float = -20.0,
        min_confidence: float = AramaKurtarmaAyarlari.MIN_CONFIDENCE,
        gereken_rov_sayisi: int | None = None,
        sessiz: bool = True,
    ) -> AlanTaramaPlani:
        self.hedef_siniflari = {str(s).lower() for s in (hedef_siniflari or [])}
        self.min_confidence = float(min_confidence)
        self.son_tespit = None
        # herkese_rota=True: her ROV kendi şeridini bağımsız, A* ile sonsuz döngüde tarar
        plan = self.alan_tarama.baslat(
            grup_id=grup_id,
            alan=alan,
            derinlik=derinlik,
            gereken_rov_sayisi=gereken_rov_sayisi,
            gorev_adi="arama_kurtarma",
            herkese_rota=True,
            sessiz=sessiz,
        )
        self.aktif_plan = plan
        # Görsel çakışmayı önlemek için sadece lider ROV'a kamera + YOLO UI ekle.
        # Diğer ROVlar bağımsız yüzer ancak kamera görüntüsü göstermez.
        kamera_rov_id = plan.lider_id
        if kamera_rov_id is None:
            kamera_rov_id = next(iter(plan.rota_by_rov), None)
        if kamera_rov_id is not None:
            if not self.filo.camera_manager.kamera_var_mi(kamera_rov_id):
                self.filo.camera_manager.kamera_ekle(rov_id=kamera_rov_id)
            self.filo.yolo_baslat(kamera_rov_id, model_path=model_path)
        return plan

    def guncelle(self) -> YoloTespit | None:
        if not self.aktif_plan:
            return None
        plan = self.aktif_plan

        # 1. YOLO tespit kontrolü — herhangi bir ROVdan hedef sınıfı görülürse dur
        for rov_id in plan.rota_by_rov:
            for raw in self.filo.camera_manager.yolo_son_tespitler.get(rov_id, []) or []:
                class_name = str(raw.get("class_name", "")).lower()
                confidence = float(raw.get("confidence", 0.0))
                if confidence < self.min_confidence:
                    continue
                if self.hedef_siniflari and class_name not in self.hedef_siniflari:
                    continue
                bbox = tuple(float(v) for v in raw.get("bbox", (0, 0, 0, 0)))
                self.son_tespit = YoloTespit(rov_id=rov_id, class_name=class_name, confidence=confidence, bbox=bbox)  # type: ignore[arg-type]
                self.durdur(lideri_takip_et=True)
                return self.son_tespit

        # 2. Yaklaşma aşamasında alan_tarama'ya devret (taramaya geçiş için)
        if plan.asama == "yaklasma":
            self.alan_tarama.guncelle(grup_id=plan.grup_id, lideri_takip_et=True)
            if plan.grup_id not in self.alan_tarama.aktif_planlar:
                self.aktif_plan = None
            return None

        # 3. Tarama aşaması — her ROV kendi şeridini bağımsız tarar (sonsuz döngü)
        # Şeridini bitiren ROV: mevcut konumundan A* ile şeridini tekrar başlat
        for rov_id in plan.rota_by_rov:
            if not self.alan_tarama._rov_tarama_bitti_mi(rov_id):
                continue
            rov = self.filo.find_rov_by_id(rov_id)
            if rov is None or getattr(rov, "is_destroyed", False):
                continue
            rov_rota = plan.rota_by_rov.get(rov_id)
            if not rov_rota:
                continue
            try:
                gps = self.filo.get(rov_id, "gps")
                bslngc: tuple[float, float] = (float(gps[0]), float(gps[1]))
            except Exception:
                bslngc = (float(rov_rota[0][0]), float(rov_rota[0][1]))
            genisletilmis = self.alan_tarama._a_star_rota_genislet(
                bslngc, rov_rota, plan.derinlik
            )
            self.filo.git(rov_id, genisletilmis, z=plan.derinlik, ai=True, sessiz=True)

        return None

    def durdur(self, lideri_takip_et: bool = True) -> None:
        plan = self.aktif_plan
        if not plan:
            return
        # Sadece lider ROV'un YOLO'sunu durdur (diğerlerine kamera eklenmedi)
        kamera_rov_id = plan.lider_id
        if kamera_rov_id is None:
            kamera_rov_id = next(iter(plan.rota_by_rov), None)
        if kamera_rov_id is not None:
            self.filo.yolo_durdur(kamera_rov_id)
        self.alan_tarama.durdur(plan.kaynak_grup_id if plan.kaynak_grup_id is not None else plan.grup_id, lideri_takip_et=lideri_takip_et)
        self.aktif_plan = None
