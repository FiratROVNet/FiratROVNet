#!/usr/bin/env python3
"""
SCHEMA/README.md içindeki 'Mevcut ROV şemaları' tablosunu otomatik üretir.
Yeni ROV klasörü (rov_motor_sema.pdf + bilgi.json ile) ekledikten sonra
bu betiği çalıştırın: python SCHEMA/update_readme.py
"""
from pathlib import Path

SCHEMA_DIR = Path(__file__).resolve().parent
README_PATH = SCHEMA_DIR / "README.md"
REQUIRED_FILES = ("rov_motor_sema.pdf", "bilgi.json")


def find_rov_folders():
    """SCHEMA altında ROV* klasörlerini bulur; gerekli dosyalar varsa listeler."""
    rovs = []
    for path in sorted(SCHEMA_DIR.iterdir()):
        if not path.is_dir():
            continue
        if path.name.startswith("_"):
            continue
        has_pdf = (path / "rov_motor_sema.pdf").is_file()
        has_json = (path / "bilgi.json").is_file()
        if has_pdf and has_json:
            rovs.append(path.name)
    return rovs


def build_table(rovs):
    lines = [
        "| ROV | Motor şeması (PDF) | Veri (JSON) |",
        "|-----|-------------------|--------------|",
    ]
    for name in rovs:
        lines.append(
            f"| {name} | "
            f"[rov_motor_sema.pdf]({name}/rov_motor_sema.pdf) | "
            f"[bilgi.json]({name}/bilgi.json) |"
        )
    return "\n".join(lines)


def main():
    rovs = find_rov_folders()
    table = build_table(rovs)

    if not README_PATH.is_file():
        print("README.md bulunamadı.")
        return
    text = README_PATH.read_text(encoding="utf-8")

    start_marker = "## Mevcut ROV şemaları"
    if start_marker not in text:
        print("README'de 'Mevcut ROV şemaları' bölümü bulunamadı.")
        return

    before = text.split(start_marker)[0]
    after_part = text.split(start_marker)[1]
    # Sonraki "## " ile başlayan bölüme kadar olan kısmı atla (tek bölüm güncellemesi)
    if "## " in after_part:
        rest = "\n## " + after_part.split("## ", 1)[1]
    else:
        rest = after_part

    new_section = (
        start_marker
        + "\n\n*Aşağıdaki tablo `update_readme.py` ile otomatik üretilir.*\n\n"
        + table
        + "\n\n---\n\n"
        + rest.lstrip("\n")
    )
    new_content = before + new_section
    README_PATH.write_text(new_content, encoding="utf-8")
    print(f"Mevcut ROV şemaları güncellendi: {rovs}")


if __name__ == "__main__":
    main()
