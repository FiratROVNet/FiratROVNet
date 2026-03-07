from .mixins.data import DataMixin
from .mixins.geometry import GeometryMixin
from .mixins.formation import FormationMixin
from .mixins.navigation import NavigationMixin
from .mixins.visualization import VisualizationMixin
from .mixins.training import TrainingMixin

class FiloHelper(DataMixin, GeometryMixin, FormationMixin, NavigationMixin, VisualizationMixin, TrainingMixin):
    """
    Filo için ana yardımcı sınıf.
    Tüm matematiksel ve operasyonel mantığı Mixin'lerden toplar.
    """
    
    VEKTOR_RENK_KODLARI = ('k', 'y', 'm', 's', 't')

    def __init__(self, filo_ref):
        self.filo = filo_ref
        
        # Mixin'lerin kullandığı ortak değişkenleri başlat
        self._vektor_baslangic = None
        self._vektor_bitis = None
        self._vektor_renk = 'm'
        self._vektor_uzunluk_metre = 10.0
        self._vektor_reverse = False
        self._apf_vektor_list = []
        self._apf_prev_vektor = {}
        self._koordinator = None
        self.kalici_hedefler = {}
        self.formasyon_sec_tekrar = 0
        
        # Async/Worker results cache sistemi
        self.last_formasyon_result = None          # Son formasyon_sec sonucu
        self.formasyon_result_timestamp = None     # Sonuçun zamanı
        self.formasyon_future: "concurrent.futures.Future | None" = None  # Active Future (worker tracking)
        
        # Hull 100 samples cache sistemi
        self.last_hull_samples = None              # Son get_100_samples sonucu (numpy array)
        self.last_hull_samples_info = None         # Meta info (nokta sayısı, alan, vs)
        self.hull_samples_timestamp = None         # Sonucun zamanı