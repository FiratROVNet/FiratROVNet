from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .alan_tarama import AlanTaramaGorevi, AlanTaramaPlani
from ...config import AramaKurtarmaAyarlari


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
        plan = self.alan_tarama.baslat(
            grup_id=grup_id,
            alan=alan,
            derinlik=derinlik,
            gereken_rov_sayisi=gereken_rov_sayisi,
            gorev_adi="arama_kurtarma",
            sessiz=sessiz,
        )
        self.aktif_plan = plan
        for rov_id in plan.rota_by_rov:
            if not self.filo.camera_manager.kamera_var_mi(rov_id):
                self.filo.camera_manager.kamera_ekle(rov_id=rov_id)
            self.filo.yolo_baslat(rov_id, model_path=model_path)
        return plan

    def guncelle(self) -> YoloTespit | None:
        if not self.aktif_plan:
            return None
        for rov_id in self.aktif_plan.rota_by_rov:
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
        self.alan_tarama.guncelle(lideri_takip_et=True)
        if self.aktif_plan and self.aktif_plan.grup_id not in self.alan_tarama.aktif_planlar:
            self.aktif_plan = None
        return None

    def durdur(self, lideri_takip_et: bool = True) -> None:
        plan = self.aktif_plan
        if not plan:
            return
        for rov_id in plan.rota_by_rov:
            self.filo.yolo_durdur(rov_id)
        self.alan_tarama.durdur(plan.kaynak_grup_id if plan.kaynak_grup_id is not None else plan.grup_id, lideri_takip_et=lideri_takip_et)
        self.aktif_plan = None
