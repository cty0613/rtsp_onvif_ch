import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

Gst.init(None)

pipeline = Gst.parse_launch("""
rtspsrc location=rtsp://192.168.0.64:554/profile2/media.smp name=src latency=0
src. ! application/x-rtp, media=video ! rtph264depay ! h264parse ! mp4mux ! filesink location=video_output.mp4
src. ! application/x-rtp, media=application ! queue ! appsink name=md_sink emit-signals=true
""")

appsink = pipeline.get_by_name("md_sink")

def on_new_sample(sink):
    sample = sink.emit("pull-sample")
    if sample:
        buf = sample.get_buffer()
        success, map_info = buf.map(Gst.MapFlags.READ)
        if success:
            data = map_info.data
            with open("metadata_output.bin", "ab") as f:
                f.write(data)
            buf.unmap(map_info)
    return Gst.FlowReturn.OK

appsink.connect("new-sample", on_new_sample)

pipeline.set_state(Gst.State.PLAYING)

# 메인 루프 실행 (SIGINT 등으로 종료)
import time
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pipeline.set_state(Gst.State.NULL)
