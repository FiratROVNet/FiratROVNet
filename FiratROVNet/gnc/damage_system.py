"""
Damage System Module
Çarpışma ve hasar hesaplama sistemi.
"""

from ursina import Vec3


class DamageSystem:
    """
    ROV çarpışma ve hasar yönetim sistemi.
    """
    
    def __init__(self, filo_ref=None):
        """
        Args:
            filo_ref: Filo referansı
        """
        self.filo_ref = filo_ref
    
    def carpisma_enerjisi_hesapla(self, m1, v1_vec, m2=None, v2_vec=None, e=0.3):
        """
        Çarpışma anında açığa çıkan hasar enerjisini (Joule) hesaplar.
        
        Args:
            m1: Birinci nesnenin kütlesi (kg)
            v1_vec: Birinci nesnenin hız vektörü (Vec3)
            m2: İkinci nesnenin kütlesi (kg, opsiyonel - sabit engel için None)
            v2_vec: İkinci nesnenin hız vektörü (Vec3, opsiyonel)
            e: Elastikiyet katsayısı (0-1 arası, varsayılan 0.3)
            
        Returns:
            float: Hasar enerjisi (Joule)
        """
        # 1. Su Altı Efektif Kütlesi (Added Mass %50)
        m1_eff = m1 * 1.5
        
        # 2. Bağıl Hız Büyüklüğünü Hesapla
        if v2_vec is None:
            v2_vec = Vec3(0, 0, 0)
            
        # İki hız vektörü arasındaki farkın uzunluğu (m/s)
        v_bagil_mag = (v1_vec - v2_vec).length()
        
        # 3. Enerji Hesabı
        if m2 is None:
            # Sabit Engel (Ada, Duvar)
            hasar_enerjisi = 0.5 * m1_eff * (v_bagil_mag**2) * (1 - e**2)
        else:
            # Hareketli Nesne (ROV - ROV)
            m2_eff = m2 * 1.5
            indirgenmis_m = (m1_eff * m2_eff) / (m1_eff + m2_eff)
            hasar_enerjisi = 0.5 * indirgenmis_m * (v_bagil_mag**2) * (1 - e**2)
        
        return round(hasar_enerjisi, 2)
    
    def rov_hasar_kontrol(self, rov, joule_esigi=10.0):
        """
        ROV'un hasar alıp almadığını kontrol eder.
        
        Args:
            rov: ROV entity
            joule_esigi: Patlama için gerekli minimum enerji (Joule)
            
        Returns:
            bool: Patlama gerektiriyorsa True
        """
        from FiratROVNet.config import Hidrodinamik
        
        if not hasattr(self.filo_ref, 'ortam_ref') or not self.filo_ref.ortam_ref:
            return False
            
        # Tüm entity'leri içeren tüm nesnelerin listesi (island_entities çakışan nesne ve engelleri içerir)
        diger_nesneler = self.filo_ref.ortam_ref.island_entities
        diger_rovlar = [r for r in self.filo_ref.ortam_ref.rovs if r and r.id != rov.id]
        
        # 1. Sabit Engel Çarpışmaları (Adalar, Kayalar)
        for entity in diger_nesneler:
            if not entity or (hasattr(entity, 'is_destroyed') and entity.is_destroyed):
                continue
                
            carpisma = rov.intersects(entity)
            if carpisma.hit:
                # Hız vektörü
                v1 = getattr(rov, 'velocity', Vec3(0, 0, 0))
                
                # Çarpışma enerjisi hesapla (m2=None → Sabit engel)
                hesaplanan_joule = self.carpisma_enerjisi_hesapla(
                    m1=Hidrodinamik.KUTLE,
                    v1_vec=v1,
                    m2=None,
                    e=0.2  # Sert çarpışma (ada/kaya)
                )
                
                if hesaplanan_joule >= joule_esigi:
                    print(f"💥 PATLAMA: ROV-{rov.id} adaya/kayaya çarptı! Enerji: {hesaplanan_joule}J (Eşik: {joule_esigi}J)")
                    return True
                    
        # 2. ROV-ROV Çarpışmaları
        for entity in diger_rovlar:
            if not entity or (hasattr(entity, 'is_destroyed') and entity.is_destroyed):
                continue
                
            carpisma = rov.intersects(entity)
            if carpisma.hit:
                v1 = getattr(rov, 'velocity', Vec3(0, 0, 0))
                v2 = getattr(entity, 'velocity', Vec3(0, 0, 0))
                
                hesaplanan_joule = self.carpisma_enerjisi_hesapla(
                    m1=Hidrodinamik.KUTLE,
                    v1_vec=v1,
                    m2=Hidrodinamik.KUTLE,
                    v2_vec=v2,
                    e=0.5  # Daha yumuşak (ROV-ROV)
                )
                
                if hesaplanan_joule >= joule_esigi:
                    print(f"💥 PATLAMA: ROV-{rov.id} ↔ ROV-{entity.id} çarpışması! Enerji: {hesaplanan_joule}J")
                    return True
                else:
                    # Hafif temas - Yavaşlatma efekti
                    yavaslatma_orani = 0.85
                    rov.velocity *= yavaslatma_orani
                    
                    if hesaplanan_joule > (joule_esigi/2):
                        print(f"🔔 Hafif Temas: ROV-{rov.id} | Enerji: {hesaplanan_joule}J (Eşik altı)")
        
        return False
    
    def rov_is_hit(self, rov_id: int):
        """
        ROV'un ada içinde olup olmadığını kontrol eder.
        
        Args:
            rov_id: ROV ID'si
            
        Returns:
            bool: ROV ada içinde ise True
        """
        if not hasattr(self.filo_ref, 'ortam_ref') or not self.filo_ref.ortam_ref:
            return False
            
        islands_and_rovs = self.filo_ref.ortam_ref.island_entities + self.filo_ref.ortam_ref.rovs
        target_rov = self.filo_ref.find_rov_by_id(rov_id)
        
        if not target_rov:
            return False
            
        for entity in islands_and_rovs:
            if entity and not (hasattr(entity, 'is_destroyed') and entity.is_destroyed):
                is_hit = target_rov.intersects(entity).hit
                if is_hit:
                    return True
        return False
    
    def rov_hasar_kontrol_direct(self, rov, joule_esigi=10.0):
        """
        ROV'un çarpışmalarını kontrol eder.
        Enerji 'joule_esigi' değerinin üzerindeyse True döner (Patlama tetiklenir).
        
        Args:
            rov: ROV entity
            joule_esigi: Hasar eşiği (Joule)
            
        Returns:
            bool: Patlama gerektiriyorsa True
        """
        from ursina import Vec3
        
        if not rov or (hasattr(rov, 'is_destroyed') and rov.is_destroyed):
            return False

        # ortam_ref guard — rov_hasar_kontrol ile tutarlı
        if not hasattr(self.filo_ref, 'ortam_ref') or not self.filo_ref.ortam_ref:
            return False

        # Çevredeki potansiyel engeller (Adalar ve Diğer ROV'lar)
        islands_and_rovs = self.filo_ref.ortam_ref.island_entities + self.filo_ref.ortam_ref.rovs
        
        for entity in islands_and_rovs:
            # Kendisiyle çarpışma kontrolü yapma ve ölü nesneleri atla
            if entity and entity != rov and not (hasattr(entity, 'is_destroyed') and entity.is_destroyed):
                
                # Ursina çarpışma testi
                hit_info = rov.intersects(entity)
                
                if hit_info.hit:
                    # 1. FİZİKSEL VERİLER
                    m1 = getattr(rov, 'mass', 12.0)
                    v1 = getattr(rov, 'velocity', Vec3(0, 0, 0))
                    
                    # Çarpılan nesne bir ROV mu yoksa sabit engel mi?
                    is_rov = hasattr(entity, 'gnc')
                    m2 = getattr(entity, 'mass', 12.0) if is_rov else None
                    v2 = getattr(entity, 'velocity', Vec3(0, 0, 0))
                    
                    # 2. ENERJİ HESABI
                    # Kaya/Ada için esneklik (e) 0.1, ROV için 0.4 (biraz daha esnek)
                    esneklik = 0.4 if is_rov else 0.1
                    hesaplanan_joule = self.carpisma_enerjisi_hesapla(m1, v1, m2, v2, e=esneklik)
                    
                    # 3. EŞİK KONTROLÜ
                    if hesaplanan_joule >= joule_esigi:
                        print(f"💥 KRİTİK HASAR! ROV-{rov.id} | Enerji: {hesaplanan_joule}J | Eşik: {joule_esigi}J")
                        return True
                    else:
                        # Hafif çarpışma: Hasar yok ama fiziksel tepki (hız kesme)
                        # Enerji eşiğe ne kadar yakınsa o kadar çok hız kaybeder
                        yavaslatma_orani = max(0.1, 1.0 - (hesaplanan_joule / joule_esigi))
                        rov.velocity *= yavaslatma_orani
                        
                        if hesaplanan_joule > (joule_esigi / 2):  # Çok küçük sürtünmeleri yazdırma
                            print(f"🔔 Hafif Temas: ROV-{rov.id} | Enerji: {hesaplanan_joule}J (Eşik altı)")
        
        return False
