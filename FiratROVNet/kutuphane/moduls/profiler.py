import time as pytime
from collections import deque
import re

class Profiler:
    # history = { "görev_adı": {"samples": deque, "parent": str or None} }
    _history = {}
    _starts = {}
    _stack = []  # Hiyerarşiyi takip eden yığın

    @staticmethod
    def start(name):
        # Eğer stack'te biri varsa, o şu anki fonksiyonun ebeveynidir
        parent_name = Profiler._stack[-1] if Profiler._stack else None
        
        # Görevi stack'e ekle
        Profiler._stack.append(name)
        
        # İlk defa karşılaşıyorsak hiyerarşiyi kaydet
        if name not in Profiler._history:
            Profiler._history[name] = {
                "samples": deque(maxlen=10),
                "parent": parent_name
            }
        
        Profiler._starts[name] = pytime.perf_counter()

    @staticmethod
    def end(name):
        if name in Profiler._starts:
            dt = pytime.perf_counter() - Profiler._starts[name]
            samples = Profiler._history[name]["samples"]
            if samples is not None:
                samples.append(dt)
            
            # Görev bittiği için stack'ten çıkar (en üstteki olması gerekir)
            if Profiler._stack and Profiler._stack[-1] == name:
                Profiler._stack.pop()
            
            # Güvenlik: Eğer yanlış end çağrılırsa temizle (opsiyonel)
            elif name in Profiler._stack:
                Profiler._stack.remove(name)

    @staticmethod
    def _natural_sort_key(s):
        return [int(text) if text.isdigit() else text.lower()
                for text in re.split('([0-9]+)', s)]

    @staticmethod
    def rapor_ver():
        print("\n" + "="*60)
        print("📊 OTOMATİK HİYERARŞİK PERFORMANS RAPORU (ms)")
        print("-"*60)
        
        toplam_sure_ms = 0
        # Tüm anahtarları doğal sıralamaya göre al
        all_keys = sorted(Profiler._history.keys(), key=Profiler._natural_sort_key)
        
        # 1. Kök görevleri (ebeveyni olmayanlar) belirle
        roots = [k for k in all_keys if Profiler._history[k]["parent"] is None]
        
        for root in roots:
            data = Profiler._history[root]
            samples = data["samples"]
            if not samples: continue
            
            avg_ms = (sum(float(s) for s in samples) / len(samples)) * 1000
            print(f"{root.ljust(40)}: {avg_ms:8.4f} ms")
            toplam_sure_ms += avg_ms
            
            # 2. Bu köke ait alt görevleri (çocukları) bul ve yazdır
            for sub in all_keys:
                sub_data = Profiler._history[sub]
                if sub_data["parent"] == root:
                    sub_samples = sub_data["samples"]
                    if not sub_samples: continue
                    sub_avg_ms = (sum(float(s) for s in sub_samples) / len(sub_samples)) * 1000
                    print(f"    ↳ {sub.ljust(36)}: {sub_avg_ms:8.4f} ms")
                    
                    # 3. Torunları desteklemek istersen (opsiyonel 3. seviye)
                    for grand in all_keys:
                        if Profiler._history[grand]["parent"] == sub:
                            g_samples = Profiler._history[grand]["samples"]
                            if not g_samples: continue
                            g_avg = (sum(float(s) for s in g_samples) / len(g_samples)) * 1000
                            print(f"        ↳ {grand.ljust(32)}: {g_avg:8.4f} ms")
            
        print("-" * 60)
        print(f"{'HESAPLANAN TOPLAM (Ana İşlemler)'.ljust(40)}: {toplam_sure_ms:8.4f} ms")
        print("="*60 + "\n")
