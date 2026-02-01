import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

Gst.init(None)

pipeline = (
    'nvarguscamerasrc ! '
    'video/x-raw(memory:NVMM),width=640,height=480,framerate=30/1 ! '
    'nvvidconv ! '
    'video/x-raw(memory:NVMM),format=I420 ! '
    'nvv4l2h264enc bitrate=1000000 ! '
    'h264parse ! '
    'rtph264pay config-interval=1 pt=96 ! '
    'udpsink host=10.47.1.70 port=3000'
)

print("Starting pipeline: ", pipeline)
pipeline = Gst.parse_launch(pipeline)
pipeline.set_state(Gst.State.PLAYING)

loop = GObject.MainLoop()
try:
    loop.run()
except KeyboardInterrupt:
    pass

pipeline.set_state(Gst.State.NULL)
#udp://@:5000
#süre 00:00da kalıyor görüntü gelmiyor
#Playback failed: No valid frames received hatası olduğu yazıyor 
