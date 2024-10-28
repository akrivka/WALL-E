#!/usr/bin/env python3

from imutils.video import VideoStream
from flask import Response
from flask import Flask
from flask import render_template
import threading
import imutils
import time
import cv2
import numpy as np
from utils.printing import ros_print

# ROS Imports
import rclpy
import cv_bridge

from rclpy.node         import Node
from sensor_msgs.msg    import Image

app = Flask(__name__)
node = None

#
#  Detector Node Class
#
class WebNode(Node):
    # Pick some colors, assuming RGB8 encoding.
    red    = (255,   0,   0)
    green  = (  0, 255,   0)
    blue   = (  0,   0, 255)
    yellow = (255, 255,   0)
    white  = (255, 255, 255)

    # Initialization.
    def __init__(self, name, app):
        super().__init__(name)
        self.outputFrame = None
        self.app = app
        self.HZ = 30
        self.bridge = cv_bridge.CvBridge()
        self.lock = threading.Lock()
        self.cap = cv2.VideoCapture(0)
        self.width = 640
        self.height = 480
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # self.cap.set(cv2.CAP_PROP_FPS,           10)
        self.angle = 167
        self.frame = np.random.random((500,500))
        # self.publish_feed()
        
        # Create a timer to keep calculating/sending commands.
        self.timer = self.create_timer(1 / self.HZ, self.publish_feed)
        ros_print(self, 'Webserver started')

    def publish_feed(self):
        # ros_print(self, self.angle)
        ret = False
        while not ret:
            ret, img = self.cap.read()
        img = cv2.resize(img, (self.width, self.height), interpolation=cv2.INTER_AREA)
        rows, cols, ch = img.shape
        width  = cols
        height = rows
        center = (width // 2, height // 2)

        # self.angle += 1
        rotation_matrix = cv2.getRotationMatrix2D(center, self.angle, 1.0)
        # with self.lock:
        img = cv2.warpAffine(img, rotation_matrix, (cols, rows))
        BORDER_LOW_X = 60
        BORDER_Y = 50
        BORDER_HIGH_X = 40
        img = img[BORDER_LOW_X:img.shape[0] - BORDER_HIGH_X, BORDER_Y:]
        self.frame = img
        # cv2.imshow('image', dst)
        # if cv2.waitKey(20) & 0xFF == 27:
        #     break
            
    # Shutdown
    def shutdown(self):
        self.destroy_node()

    @app.route("/")
    def index():
        # return the rendered template
        return render_template("index.html")
                
    def generate(self):
        while True:
            time.sleep(1 / self.HZ)
            # check if the output frame is available, otherwise skip
            # the iteration of the loop
            if self.frame is None:
                continue
            # encode the frame in JPEG format
            # with self.lock:
            (flag, encodedImage) = cv2.imencode(".jpg", self.frame)
            # ensure the frame was successfully encoded
            if not flag:
                continue
            # yield the output frame in the byte format
            yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
                bytearray(encodedImage) + b'\r\n')

    @app.route("/feed1")
    def feed1():
        # return the response generated along with the specific media
        # type (mime type)
        return Response(node.generate(),
            mimetype = "multipart/x-mixed-replace; boundary=frame")


def start_app(_):
    global app
    app.run(host='0.0.0.0', port=5000, debug=False,
            threaded=True, use_reloader=False)

#
#   Main Code
#
def main(args=None):
    global node
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = WebNode('webserver', app)
    t = threading.Thread(target=start_app, args=(None,))
    t.daemon = True
    t.start()
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()