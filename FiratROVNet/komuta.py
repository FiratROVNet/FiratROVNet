import numpy as np

class KomutaMerkezi:
    """
    Kullanıcıdan (Terminalden) gelen emirleri alır, doğrular
    ve ilgili GNC (Otopilot) sistemine iletir.
    """
    def __init__(self, gnc_listesi):
        # Main.py'den gelen aktif GNC sistemlerini burada saklıyoruz
        self.gnc_sistemleri = gnc_listesi

    def git(self, rov_id, x, y, z):
        """
        KONSOL KOMUTU: Belirtilen ROV'a güvenli hedef atar.
        Kullanım: git(0, 10, 50, -5) -> ROV 0'ı (10, 50) noktasına ve 5m derinliğe gönder.
        """
        # 1. Güvenlik Kontrolü: ID geçerli mi?
        if not isinstance(rov_id, int) or rov_id < 0 or rov_id >= len(self.gnc_sistemleri):
            print(f"❌ [KOMUTA] HATA: Geçersiz ROV ID ({rov_id}). Mevcut: 0-{len(self.gnc_sistemleri)-1}")
            return

        # 2. Bilgilendirme
        print(f"🤖 [KOMUTA] Emir Alındı -> ROV-{rov_id} Hedef: X={x}, Y={y}, Derinlik={z}")
        
        # 3. Emri İlet (Abstraction)
        # GNC sistemi, bu koordinatları Ursina dünyasına çevirmeyi ve GAT ile konuşmayı kendi halleder.
        self.gnc_sistemleri[rov_id].hedef_atama(x, y, z)

    def dur(self, rov_id):
        """Acil durum durdurma komutu."""
        if 0 <= rov_id < len(self.gnc_sistemleri):
            print(f"🛑 [KOMUTA] ROV-{rov_id} DURDURULUYOR.")
            # Hedefi iptal et (None yap)
            self.gnc_sistemleri[rov_id].hedef_nokta = None
            # Fiziksel olarak durdur
            self.gnc_sistemleri[rov_id].rov.velocity = np.array([0,0,0]) # Ursina Vec3 uyumu gerekebilir ama mantık bu
