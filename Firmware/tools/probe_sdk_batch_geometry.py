"""Target-free numerical equivalence/performance probe using the installed SDK.

Does not construct/open a camera or device. Compare integer outputs across crops,
ROIs, sensor sizes and off-screen boxes, including the one-pixel anchor box.
"""
import json
import time
import numpy as np
from libcamera import Rectangle
from picamera2.devices.imx500.imx500 import IMX500
try:
    from perception.model.sdk_geometry import Imx500FrameMapper
except ModuleNotFoundError:
    from sdk_geometry import Imx500FrameMapper  # standalone probe before deployment


class Configuration:
    def __init__(self, raw): self.raw=raw
    def camera_configuration(self):
        return {'main':{'size':(1920,1080)},'raw':{'size':self.raw}}


def main():
    rng=np.random.default_rng(71)
    low=rng.uniform(-.2,1.2,(500,2)); size=rng.uniform(.00001,.8,(500,2))
    boxes=np.column_stack((low,low+size)).tolist()
    boxes += [[0,0,1,1],[0,0,0,0],[-.01,-.01,.01,.01],[.5,.5,.5+1/320,.5+1/320]]
    device=object.__new__(IMX500); device.device_fd=None
    tested=0; times=[]
    for raw in ((2028,1520),(4056,3040)):
        camera=Configuration(raw)
        for roi in (None,Rectangle(0,0,0,0),Rectangle(506,0,3040,3040)):
            device._IMX500__cfg={} if roi is None else {'roi':roi}
            for crop in ((0,0,4056,3040),(0,380,4056,2280),(400,500,3000,2000)):
                metadata={'ScalerCrop':crop}
                start=time.perf_counter_ns()
                expected=[device.convert_inference_coords(b,metadata,camera) for b in boxes]
                middle=time.perf_counter_ns()
                actual=Imx500FrameMapper(device,camera,metadata).convert_many(boxes)
                end=time.perf_counter_ns()
                assert actual==expected,next((i,a,b) for i,(a,b) in enumerate(zip(actual,expected)) if a!=b)
                tested+=len(boxes)
                times.append(dict(sdk_ms=(middle-start)/1e6,batch_ms=(end-middle)/1e6))
    print(json.dumps(dict(exact_matches=tested,comparisons=times),indent=2))


if __name__=='__main__': main()
