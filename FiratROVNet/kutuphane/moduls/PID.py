import math

class PID:
    
    def __init__(self, Kp:float=0, Ki:float=0, Kd:float=0, out_min:float=-1, out_max:float=1):
        self.error = None
        self.integrate_error = 0
        self.last_error = None  # D terimi için ayrı bir değişken

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.out_min = out_min
        self.out_max = out_max
        self.integral_limit = 10  # Anti-windup için integral sınırı


    def P(self, error):
        return error * self.Kp

    def I(self, dt, error):
        # Integral birikimi
        self.integrate_error += (dt * error)
        
        # Anti-windup: integrali sınırla
        self.integrate_error = max(min(self.integrate_error, self.integral_limit), -self.integral_limit)
        
        return self.integrate_error * self.Ki

    def D(self, dt, error):
        # İlk çağrıda derivative hesaplama
        if self.last_error is None:
            self.last_error = error
            return 0
        
        # Derivative hesaplama (error üzerinden değil, değişim üzerinden)
        derivative = (error - self.last_error) / dt
        self.last_error = error
        
        return derivative * self.Kd

    def normalize(self, value):
        """-1..1 aralığına sıkıştır"""
        return math.tanh(value)

    def clamp(self, value):
        """Çıktıyı belirlenen min/max aralığına sıkıştır"""
        return max(min(value, self.out_max), self.out_min)

    def compute(self, hedef, durum, dt, normalize=False):
        """PID toplam çıktısını hesaplar"""
        # Hata hesapla
        error = hedef - durum
        
        # İlk error'u kaydet (I ve D için)
        if self.error is None:
            self.error = error
        
        # PID terimlerini hesapla
        p = self.P(error)
        i = self.I(dt, error)
        d = self.D(dt, error)
        
        # Toplam çıktı (P + I + D)
        output = p + i + d

        # İsteğe bağlı normalize et
        if normalize:
            output = self.normalize(output)
        else:
            output = self.clamp(output)
        
        # Hata değerini güncelle
        self.error = error
        
        return output
    
    def reset(self):
        """PID'nin iç durumunu sıfırla"""
        self.error = None
        self.last_error = None
        self.integrate_error = 0

