import time as pytime
from collections import deque
import re

class Profiler:
    # history = { "görev_adı": {"samples": deque, "parent": str or None} }
    _history = {}
    _starts = {}
    _stack = []  # Hiyerarşiyi takip eden yığın
    _last_auto_report = 0.0
    enabled = True
    sample_window = 120

    @staticmethod
    def start(name):
        if not Profiler.enabled:
            return
        # Eğer stack'te biri varsa, o şu anki fonksiyonun ebeveynidir
        parent_name = Profiler._stack[-1] if Profiler._stack else None
        
        # Görevi stack'e ekle
        Profiler._stack.append(name)
        
        # İlk defa karşılaşıyorsak hiyerarşiyi kaydet
        if name not in Profiler._history:
            Profiler._history[name] = {
                "samples": deque(maxlen=Profiler.sample_window),
                "parent": parent_name,
                "total": 0.0,
                "count": 0,
                "max": 0.0,
                "last": 0.0,
            }
        
        Profiler._starts.setdefault(name, []).append(pytime.perf_counter())

    @staticmethod
    def end(name):
        if not Profiler.enabled:
            return
        starts = Profiler._starts.get(name)
        if starts:
            dt = pytime.perf_counter() - starts.pop()
            if not starts:
                Profiler._starts.pop(name, None)
            samples = Profiler._history[name]["samples"]
            if samples is not None:
                samples.append(dt)
            Profiler._history[name]["total"] += dt
            Profiler._history[name]["count"] += 1
            Profiler._history[name]["last"] = dt
            if dt > Profiler._history[name]["max"]:
                Profiler._history[name]["max"] = dt
            
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
    def reset():
        Profiler._history.clear()
        Profiler._starts.clear()
        Profiler._stack.clear()
        Profiler._last_auto_report = 0.0

    @staticmethod
    def discard_matching(predicate):
        """Belirli profiler bloklarını geçmişten ve bekleyen ölçümlerden çıkarır."""
        if not callable(predicate):
            return
        for name in list(Profiler._history.keys()):
            try:
                matched = bool(predicate(name))
            except Exception:
                matched = False
            if matched:
                Profiler._history.pop(name, None)
                Profiler._starts.pop(name, None)
                Profiler._stack = [item for item in Profiler._stack if item != name]

    @staticmethod
    def discard_rov(rov_id):
        """Silinen ROV'a ait detay profiler satırlarını HUD/rapordan kaldırır."""
        try:
            rid = int(rov_id)
        except Exception:
            return
        pattern = re.compile(rf"(?<!\d)ROV-{rid}(?!\d)")
        Profiler.discard_matching(lambda name: pattern.search(str(name)) is not None)

    @staticmethod
    def snapshot():
        rows = []
        for name, data in Profiler._history.items():
            samples = data.get("samples")
            if not samples:
                continue
            avg = sum(float(s) for s in samples) / len(samples)
            total_window = sum(float(s) for s in samples)
            rows.append({
                "name": name,
                "parent": data.get("parent"),
                "avg": avg,
                "last": data.get("last", 0.0),
                "max": data.get("max", 0.0),
                "window_total": total_window,
                "count": data.get("count", 0),
            })
        return rows

    @staticmethod
    def darboğaz_raporu(top_n=15, fps=None):
        rows = Profiler.snapshot()
        if not rows:
            return
        rows = sorted(rows, key=lambda r: (r["avg"], r["window_total"]), reverse=True)
        baslik = f"📊 CANLI DARBOĞAZ RAPORU | Top {top_n}"
        if fps is not None:
            baslik += f" | FPS: {fps:.1f}"
        print("\n" + "=" * 96)
        print(baslik)
        print("-" * 96)
        print(f"{'Blok'.ljust(48)} {'Ort(ms)':>10} {'Son(ms)':>10} {'Max(ms)':>10} {'N':>7} {'Parent'}")
        print("-" * 96)
        for row in rows[:max(1, int(top_n))]:
            print(
                f"{row['name'][:48].ljust(48)} "
                f"{row['avg'] * 1000:10.3f} "
                f"{row['last'] * 1000:10.3f} "
                f"{row['max'] * 1000:10.3f} "
                f"{row['count']:7d} "
                f"{row.get('parent') or '-'}"
            )
        print("=" * 96 + "\n")

    @staticmethod
    def auto_report(interval_s=5.0, top_n=15, fps=None):
        now = pytime.perf_counter()
        if Profiler._last_auto_report <= 0.0:
            Profiler._last_auto_report = now
            return
        if now - Profiler._last_auto_report >= max(0.5, float(interval_s)):
            Profiler._last_auto_report = now
            Profiler.darboğaz_raporu(top_n=top_n, fps=fps)

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
