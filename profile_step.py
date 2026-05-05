import argparse
import cProfile
import os
import pstats
import subprocess
import sys
import time as pytime

import numpy as np

os.environ.setdefault("RR_IP_ADRESI", "127.0.0.1")

from ursina import Vec3  # type: ignore[import]

from FiratROVNet.config import PerformansAyarlari
from FiratROVNet.gnc import Filo
from FiratROVNet.kutuphane.moduls.profiler import Profiler
from FiratROVNet.simulasyon import Ortam


class FrameScheduler:
    def __init__(self):
        self._accum = {}

    def due(self, name, hz, dt, first=True):
        if hz is None or hz <= 0:
            return False
        interval = 1.0 / float(hz)
        current = self._accum.get(name)
        if current is None:
            self._accum[name] = 0.0
            return bool(first)
        current += dt
        if current >= interval:
            self._accum[name] = current % interval
            return True
        self._accum[name] = current
        return False


def _build_update(app, filo, mode, rerun_runtime=None):
    scheduler = FrameScheduler()
    tahminler = np.zeros(len(app.rovs), dtype=int)
    step = {"value": 0}
    visuals_enabled = mode == "visuals_on"

    def custom_update():
        nonlocal tahminler
        dt = 0.016
        try:
            from ursina import time as utime  # type: ignore[import]
            dt = getattr(utime, "dt", 0.016) or 0.016
        except Exception:
            pass

        if tahminler.shape[0] != len(app.rovs):
            tahminler = np.zeros(len(app.rovs), dtype=int)

        if scheduler.due("gat", PerformansAyarlari.GAT_HZ, dt):
            Profiler.start("0_guncelle_gat_analizi")
            tahminler.fill(0)
            filo.guncelle_gat_analizi(tahminler)
            Profiler.end("0_guncelle_gat_analizi")

        guncelle_gorseller = visuals_enabled and scheduler.due("gorseller", PerformansAyarlari.GORSELLER_HZ, dt)
        guncelle_lider = scheduler.due("lider", PerformansAyarlari.LIDER_HZ, dt)
        filo.guncelle_hepsi(tahminler, guncelle_gorseller=guncelle_gorseller, guncelle_lider=guncelle_lider)

        if rerun_runtime and scheduler.due("rerun", PerformansAyarlari.RERUN_HZ, dt):
            from rerun_ayarla import rerun_sahne_logla

            Profiler.start("0_rerun_sahne_logla")
            rerun_sahne_logla(app=app, filo=filo, step=step["value"])
            Profiler.end("0_rerun_sahne_logla")
        step["value"] += 1

    return custom_update


def run_case(args, mode, rerun_on=False, minimap_on=False, fpv_on=False):
    print(f"\n=== CASE mode={mode} rerun={rerun_on} minimap={minimap_on} fpv={fpv_on} rovs={args.rovs} ===")
    app = Ortam()
    app.sim_olustur(n_rovs=(args.rovs,), n_islands=args.islands, havuz_genisligi=200, rov_model=args.rov_model)
    filo = Filo(ortam_ref=app)

    if minimap_on and getattr(app, "minimap", None):
        app.minimap.goster(True)
    elif getattr(app, "minimap", None):
        app.minimap.goster(False)

    if fpv_on:
        filo.kamera_ayarla(rov_id=0)
    else:
        filo.camera_manager.tum_kameralari_kaldir()

    rerun_runtime = None
    if rerun_on:
        from rerun_ayarla import rerun_baslat

        rerun_runtime = rerun_baslat(ip_adresi="127.0.0.1")

    app.set_update_function(_build_update(app, filo, mode, rerun_runtime=rerun_runtime))

    for _ in range(args.warmup):
        app.app.step()

    pr = cProfile.Profile()
    start = pytime.perf_counter()
    pr.enable()
    for _ in range(args.frames):
        app.app.step()
    pr.disable()
    elapsed = pytime.perf_counter() - start

    print(f"Elapsed: {elapsed:.3f}s | avg frame: {(elapsed / args.frames) * 1000:.2f}ms | FPS: {args.frames / elapsed:.1f}")
    pstats.Stats(pr).sort_stats("cumtime").print_stats(args.rows)
    Profiler.rapor_ver()


def parse_args():
    parser = argparse.ArgumentParser(description="FiratROVNet frame profiler")
    parser.add_argument("--frames", type=int, default=1000)
    parser.add_argument("--warmup", type=int, default=30)
    parser.add_argument("--rows", type=int, default=35)
    parser.add_argument("--rovs", type=int, default=7)
    parser.add_argument("--islands", type=int, default=4)
    parser.add_argument("--rov-model", default="submarine")
    parser.add_argument("--mode", choices=("visuals_on", "visuals_off", "matrix"), default="visuals_on")
    parser.add_argument("--rerun", action="store_true")
    parser.add_argument("--minimap", action="store_true")
    parser.add_argument("--fpv", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if args.mode == "matrix":
        base = [
            sys.executable, __file__,
            "--frames", str(args.frames),
            "--warmup", str(args.warmup),
            "--rows", str(args.rows),
            "--rovs", str(args.rovs),
            "--islands", str(args.islands),
            "--rov-model", args.rov_model,
        ]
        cases = [
            ["--mode", "visuals_off"],
            ["--mode", "visuals_on"],
            ["--mode", "visuals_on", "--fpv"],
        ]
        if args.minimap:
            cases[1].append("--minimap")
            cases[2].append("--minimap")
        if args.rerun:
            case = ["--mode", "visuals_on", "--rerun"]
            if args.minimap:
                case.append("--minimap")
            if args.fpv:
                case.append("--fpv")
            cases.append(case)
        for case in cases:
            subprocess.run(base + case, check=False)
    else:
        run_case(args, args.mode, rerun_on=args.rerun, minimap_on=args.minimap, fpv_on=args.fpv)
