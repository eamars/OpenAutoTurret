import json
from types import SimpleNamespace
from picamera2.devices.imx500.imx500 import IMX500
from perception.model.imx500_yolo import Imx500YoloAdapter
from perception.config import AnchorConfig

# Invoke the installed SDK's actual rectangle/crop conversion with the stream
# dimensions and ScalerCrop captured on this station. This owns no camera.
class CapturedConfiguration:
    def camera_configuration(self):
        return {'main':{'size':(1920,1080)},'raw':{'size':(2028,1520)}}

sdk=object.__new__(IMX500)
sdk.device_fd=None
sdk._IMX500__cfg={}
camera=CapturedConfiguration()
metadata={'ScalerCrop':(0,380,4056,2280)}
rows=[]
adapter=object.__new__(Imx500YoloAdapter)
adapter._stream=(1920,1080)
adapter.device=sdk
adapter.camera=camera
adapter.anchor_cfg=AnchorConfig()
adapter.manifest=SimpleNamespace(input_width=320,input_height=320,
    bbox_order='yxyx',bbox_normalized=True,score_indices=lambda:(0,1,2))
for shift in [-.1,-.05,0,.05,.1]:
    box=(.1+shift,.3,.9+shift,.7)
    x,y,w,h=sdk.convert_inference_coords(box,metadata,camera)
    native_y=box[0]+.45*(box[2]-box[0])
    expected_y=(native_y*3040-380)/2280*1080
    old_y=y+.45*h
    _, anchors=adapter._map_rows_to_stream([[.8,0,*box]],metadata)
    corrected=anchors[0][0].y*1080
    assert anchors[0][1]
    assert abs(corrected-expected_y)<2
    rows.append({'native_box_y':[box[0],box[2]],'sdk_visible_box':[x,y,w,h],
        'post_crop_anchor_y':old_y,'full_box_anchor_y':expected_y,'bias_px':old_y-expected_y,
        'corrected_anchor_y':corrected,'corrected_bias_px':corrected-expected_y})
print(json.dumps(rows,indent=2))
