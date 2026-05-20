"""
Logging and Error Tracking Module
Hata izleme ve günlüğe kayıt sistemi
"""

import inspect
import os
import traceback
import logging


class LogSystem:
    """
    Sistem log ve hata takip sistemi.
    """
    
    @staticmethod
    def get_location_info():
        """
        Dosya ve satır bilgisini döndürür - çağrıldığı yer
        Returns: "dosya:satir" formatında string
        """
        frame = inspect.currentframe().f_back.f_back
        dosya = os.path.basename(frame.f_code.co_filename)
        satir = frame.f_lineno
        return f"{dosya}:{satir}"
    
    @staticmethod
    def log_exception(e):
        """
        Exception'ı ayrıntılı olarak loglalar.
        
        Args:
            e: Hata objesi (Exception)
        """
        # 1. Hata İzleme (Exception Traceback)
        tb_list = traceback.extract_tb(e.__traceback__)
        
        print(f"\n--- HATA DETAYLARI ---")
        print(f"Hata Mesajı: {e}")

        if tb_list:
            hata_frame = tb_list[-1]  # Hatayı veren yer
            print(f"Hata Veren Fonksiyon: {hata_frame.name} (Satır: {hata_frame.lineno})")
            print(f"Hata Kodu: {hata_frame.line}")
        else:
            print("Hata Traceback: Mevcut değil (exception doğrudan raise edildi)")

        # 2. Hatayı Veren Fonksiyonu Çağıran (Hata Zinciri içindeki bir üst)
        if len(tb_list) > 1:
            zincir_cagiran = tb_list[-2]
            print(f"Hata Zincirindeki Çağırıcı: {zincir_cagiran.name} (Dosya: {zincir_cagiran.filename})")

        # 3. Setter'ı Gerçekten Çağıran
        stack = traceback.extract_stack()
        if len(stack) >= 3:
            gercek_cagiran = stack[-3]
            print(f"Setter'ı Tetikleyen Yer: {gercek_cagiran.name} (Dosya: {gercek_cagiran.filename}, Satır: {gercek_cagiran.lineno})")
    
    @staticmethod
    def log_call_stack(dosya_bilgisi_ver=True):
        """
        Çağrı zincirini (Call Stack) çıkarır.
        Format: fonk1(dosya:satir) --> fonk2(dosya:satir) --> ...
        
        Args:
            dosya_bilgisi_ver: Dosya bilgisini de içer mi?
            
        Returns:
            str: Çağrı zinciri
        """
        try:
            zincir = []
            cerceve = inspect.currentframe().f_back
            
            while cerceve:
                fonk_adi = cerceve.f_code.co_name
                
                # Dosya yolunu sadece isim olarak al
                tam_yol = cerceve.f_code.co_filename
                dosya_adi = os.path.basename(tam_yol)
                satir = cerceve.f_lineno

                if fonk_adi == '<module>':
                    fonk_adi = "ANA_DIZIN"

                # Format
                if dosya_bilgisi_ver:
                    bilgi = f"{fonk_adi}({dosya_adi}:{satir})"
                else:
                    bilgi = fonk_adi
                
                zincir.append(bilgi)
                cerceve = cerceve.f_back

            # Listeyi başa doğru (akış sırasına göre) çeviriyoruz
            zincir.reverse()
            
            oklu_yol = " --> ".join(zincir)
            
            # Hem loga hem konsola basalım
            logging.debug(f"İZLEME: {oklu_yol}")
            print(f"\n[İZLEME]: {oklu_yol}\n")
            
            return oklu_yol
            
        except Exception as e:
            LogSystem.log_exception(e)
            return str(e)
