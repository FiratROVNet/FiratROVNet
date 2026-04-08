import rerun as rr
import numpy as np
import time

rr.init("web_ornek")
rr.serve_grpc()

print("Veri akışı başlıyor...")

for i in range(100):
    rr.set_time_sequence("step", i)
    rr.log("points", rr.Points3D(np.random.randn(10, 3)))
    time.sleep(0.1)

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass
