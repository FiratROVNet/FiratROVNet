#!/usr/bin/env python3
"""Debug: Ortam.app'ın Panda3D scene graph'ını dump et."""

import sys
sys.path.insert(0, '.')

from FiratROVNet.simulasyon import Ortam

print("🔵 Ortam instance oluşturuluyor...")
app = Ortam()

print("\n📊 Panda3D Scene Graph Dump (sim_olustur() ÖNCESI):")
print("=" * 60)

def dump_node_tree(node, indent=0):
    """Panda3D node tree'yi recursive dump et."""
    prefix = "  " * indent
    try:
        node_name = node.getName() if hasattr(node, 'getName') else str(node)
        node_type = node.__class__.__name__ if hasattr(node, '__class__') else type(node)
        node_scale = f" [scale={node.getScale()}]" if hasattr(node, 'getScale') else ""
        print(f"{prefix}{node_type}: {node_name}{node_scale}")
    except Exception as e:
        print(f"{prefix}[Error: {e}]")
        return
    
    # Children varsa recurse
    if hasattr(node, 'getChildren'):
        try:
            children = node.getChildren()
            for child in children:
                dump_node_tree(child, indent + 1)
        except Exception:
            pass

try:
    render = app.app.render
    print(f"Root node: {render}")
    dump_node_tree(render)
except Exception as e:
    print(f"Error accessing render: {e}")

print("\n" + "=" * 60)
print("🔧 sim_olustur() çağrılıyor...")
print("=" * 60)

app.sim_olustur(n_rovs=(0,), n_islands=1, havuz_genisligi=200, rov_model='submarine')

print("\n📊 Panda3D Scene Graph Dump (sim_olustur() SONRASI):")
print("=" * 60)

try:
    render = app.app.render
    dump_node_tree(render)
except Exception as e:
    print(f"Error accessing render: {e}")

print("\n" + "=" * 60)
print("🌍 Ortam entities:")
print(f"  water_volume: {getattr(app, 'water_volume', None)}")
print(f"  ocean_surface: {getattr(app, 'ocean_surface', None)}")
print(f"  ocean_taban: {getattr(app, 'ocean_taban', None)}")
print(f"  seabed: {getattr(app, 'seabed', None)}")
print(f"  cimen_katmani: {getattr(app, 'cimen_katmani', None)}")
print(f"  Total ROVs: {len(app.rovs)}")

print("\n" + "=" * 60)
print("✅ Debug takas hakkında bilgi toplandı.")
print("Lütfen 'scale' değerlerini ve duplicate node'ları kontrol et.")
