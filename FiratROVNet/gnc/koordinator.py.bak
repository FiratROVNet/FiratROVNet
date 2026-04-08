"""
Koordinator Module
Simülasyon ve Ursina koordinat sistemleri arası dönüşümler.
"""


class Koordinator:
    """
    Simülasyon (X:Sağ, Y:İleri, Z:Derinlik) <-> Ursina (X, Y:Yukarı, Z:İleri) dönüşümü.
    """
    
    @staticmethod
    def sim_to_ursina(sim_x, sim_y, sim_z):
        """
        Simülasyon koordinatlarını Ursina koordinatlarına dönüştürür.
        
        Args:
            sim_x: Simülasyon X koordinatı (Sağ)
            sim_y: Simülasyon Y koordinatı (İleri)
            sim_z: Simülasyon Z koordinatı (Derinlik, negatif = aşağı)
            
        Returns:
            tuple: (ursina_x, ursina_y, ursina_z)
        """
        from FiratROVNet.kutuphane.helper.simulasyon_helper import sim_to_ursina as _stou
        return _stou(sim_x, sim_y, sim_z)

    @staticmethod
    def ursina_to_sim(u_x, u_y, u_z):
        """
        Ursina koordinatlarını simülasyon koordinatlarına dönüştürür.
        
        Args:
            u_x: Ursina X koordinatı
            u_y: Ursina Y koordinatı (Yukarı)
            u_z: Ursina Z koordinatı (İleri)
            
        Returns:
            tuple: (sim_x, sim_y, sim_z)
        """
        from FiratROVNet.kutuphane.helper.simulasyon_helper import ursina_to_sim as _utot
        return _utot(u_x, u_y, u_z)


class SafeDict(dict):
    """
    None döndüren güvenli sözlük.
    Eksik anahtarlar için istisna fırlatmaz.
    """
    def __missing__(self, key):
        return None
