# SCHEMA — ROV Şema ve Dokümanlar

Bu klasör, ROV motor yerleşimi ve diğer yapısal şemaların (PNG/PDF) kaydedildiği yerdir. Her ROV’a ait çıktılar **ROVid** (örn. `ROV0`, `ROV1`) alt klasöründe tutulur.

## Klasör yapısı

```
SCHEMA/
├── ROV0/
│   ├── bilgi.json           # ROV id ve motor listesi (konum, açı)
│   └── rov_motor_sema.pdf   # Teknik çizim: üst + yan görünüm
├── ROV1/
│   └── ...
└── README.md
```

## Motor şeması

- **Üst görünüm (XZ):** ROV yukarıdan; X = sol/sağ, Z = ön/arka. Motor konumları ve itiş yönleri (ok) ile açılar (rx, ry, rz °) gösterilir.
- **Yan görünüm (ZY):** ROV yandan; Z = ön/arka, Y = aşağı/yukarı.

### Oluşturma

Simülasyon çalışırken (Filo ve ROV’lar oluşturulduktan sonra):

```python
# İlk ROV → SCHEMA/ROV0/
filo = ortam.filo
filo.motor_sema_kaydet()

# Belirli ROV → SCHEMA/ROV{id}/
filo.motor_sema_kaydet(rov=ortam.rovs[1])  # ROV1 klasörüne yazar

# Özel üst klasör (çıktı yine klasor/ROV{id}/ olur)
filo.motor_sema_kaydet(klasor="SCHEMA", base_name="rov_motor_sema")
```

Her ROV için oluşan dosyalar (`SCHEMA/ROV{id}/` içinde):

- `bilgi.json` — rov_id ve motorlar (name, position, rotation)
- `rov_motor_sema.pdf` — tek PDF; üstte üst görünüm (XZ), altta yan görünüm (ZY) (teknik çizim mantığı)

### Motor ID eşlemesi (m0–m5)

| ID  | Konum      | Açıklama   |
|-----|------------|------------|
| m0  | Ön sol     | Yatay      |
| m1  | Ön sağ     | Yatay      |
| m2  | Arka sol   | Yatay      |
| m3  | Arka sağ   | Yatay      |
| m4  | Dikey sol  | Heave      |
| m5  | Dikey sağ  | Heave      |

Bu klasöre ileride ek doküman ve şemalar eklenebilir.
