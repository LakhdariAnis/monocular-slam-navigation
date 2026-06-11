import cv2
import zmq
import numpy as np
from picamera2 import Picamera2
import libcamera

# --- Config ---
PORT = 5555
RESOLUTION = (640, 480)
JPEG_QUALITY = 80

picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": RESOLUTION, "format": "BGR888"},
    raw={"size": (1640, 1232)},
    transform=libcamera.Transform(hflip=1, vflip=1)
)
picam2.configure(config)
picam2.start()

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind(f"tcp://*:{PORT}")

print(f"Streaming on port {PORT} — Ctrl+C to stop")

try:
    while True:
        frame = picam2.capture_array()
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        socket.send(buf.tobytes())

except KeyboardInterrupt:
    print("Stopped.")
finally:
    picam2.stop()
    socket.close()
    context.term()

