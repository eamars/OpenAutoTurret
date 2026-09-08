"""Batch the installed IMX500 SDK's rectangle conversion, preserving integer clipping.

The IMX500 active sensor is 4056 x 3040. The operations below match Picamera2's
convert_inference_coords / __get_obj_scaled, using libcamera's own Rectangle
operations. The adapter retains the SDK path for other devices. Exercise
probe_sdk_batch_geometry.py against the installed SDK before deployment.
"""
import numpy as np


class Imx500FrameMapper:
    def __init__(self, device, camera, metadata):
        from libcamera import Rectangle, Size
        self.Rectangle = Rectangle
        self.Size = Size
        config = camera.camera_configuration()
        self.full = Rectangle(0,0,4056,3040)
        self.output = Size(*config['main']['size'])
        self.sensor = Size(*config['raw']['size'])
        self.crop = Rectangle(*metadata['ScalerCrop']).scaled_by(self.sensor,self.full.size)
        self.roi = device.config.get('roi')
        if self.roi == Rectangle(0,0,0,0):
            self.roi = None

    def convert_many(self, coordinates):
        if not coordinates:
            return []
        # Retain the SDK's truncation and nonnegative rectangle components.
        a=np.asarray(coordinates,dtype=np.float64)
        rects=np.column_stack((a[:,1]*4056,a[:,0]*3040,
                               (a[:,3]-a[:,1])*4056,(a[:,2]-a[:,0])*3040))
        rects=np.maximum(rects,0).astype(np.int32)
        output=[]
        for values in rects:
            obj=self.Rectangle(*values)
            if self.roi is not None:
                obj=obj.bounded_to(self.roi)
            obj=obj.scaled_by(self.sensor,self.full.size).bounded_to(self.crop)
            obj=obj.translated_by(-self.crop.topLeft).scaled_by(self.output,self.crop.size)
            output.append(obj.to_tuple())
        return output
