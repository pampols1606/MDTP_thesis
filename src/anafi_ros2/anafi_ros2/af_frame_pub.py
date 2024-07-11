#---------------
# IMPORTS
#----------------
import rclpy
import rclpy.clock
from rclpy.node import Node
from anafi_msg.msg import ControlData, PnPData
import rclpy.time
from sensor_msgs.msg import Image
from pynput.keyboard import Listener

import numpy as np
import cv2, os, olympe, time, threading, queue
from cv_bridge import CvBridge

import warnings

warnings.simplefilter("ignore")
olympe.log.update_config({"loggers": {"olympe": {"level": "CRITICAL"}}})

WiFi_SSID = 'ANAFI-H073380'

#---------------
# NODE CLASS
#----------------
class FramePub(Node):
    def __init__(self):
        super().__init__('af_frame_pub')
        self.image_pub = self.create_publisher(Image, 'anafi/frames', 1000)
        self.listener = Listener(on_press=self.on_press)
        self.listener.start()

        self.bridge = CvBridge()
        self.frame_queue = queue.LifoQueue()
        self.processing_thread = threading.Thread(target=self.yuv_frame_processing)
        self.cv2_cvt_color_flag = {
                    olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
                    olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
                }
        self.Running = True
        self.Connected = False
        self.frameid = 0

        cv2.namedWindow('Display raw', cv2.WINDOW_NORMAL)

    def Connect(self):
        self.get_logger().info('Connecting to Anafi drone...')

        self.DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")
        self.DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT")
        self.drone = olympe.Drone(self.DRONE_IP)

        for i in range(5):
            connected = self.drone.connect(retry=1)
            if connected:
                self.Connected = True
                self.get_logger().info('Conected to Anafi drone!')
                time.sleep(0.5)
                break
            else:
                self.get_logger().info(f'Trying to connect ({i+1})')
                time.sleep(2)
            
        if self.Connected:
            if self.DRONE_RTSP_PORT is not None:
                self.drone.streaming.server_addr = f"{self.DRONE_IP}:{self.DRONE_RTSP_PORT}"
            
            self.drone.streaming.set_callbacks(
                raw_cb=self.yuv_frame_cb,
                flush_raw_cb=self.flush_cb,)
        
            self.drone.streaming.start()
            self.processing_thread.start()
        else:
            self.get_logger().info("Failed to connect.")
    
    def Stop(self):
        
        self.Running = False
        if self.Connected:
            self.processing_thread.join(timeout=1.0)
            self.drone.streaming.stop()
            self.drone.disconnect()

        self.get_logger().info('Shutting down...\n')
        self.image_pub.destroy()
        self.listener.stop()
        self.Connected = False
        time.sleep(0.2)
        self.destroy_node()
        rclpy.shutdown()

    def yuv_frame_cb(self, yuv_frame):
        
        try:
            yuv_frame.ref()
            self.frame_queue.put_nowait(yuv_frame)

        except Exception as e:
            self.get_logger().info(f"Error handling media removal: {e}")


    def yuv_frame_processing(self):
        t=0
        prev_t = 0
        while self.Running:
            try:
                t = (self.get_clock().now().nanoseconds)/1000000000
                yuv_frame = self.frame_queue.get(timeout=0.1)
                
                if yuv_frame is not None:
                    x = yuv_frame.as_ndarray()
                    cv2frame = cv2.cvtColor(x, self.cv2_cvt_color_flag[yuv_frame.format()])
                    cv2.imshow('Display raw', cv2frame)
                    cv2.waitKey(1)
                    msg = self.bridge.cv2_to_imgmsg(cv2frame, "bgr8")
                    msg.header.frame_id = str(self.frameid)
                    self.image_pub.publish(msg)
                    self.frameid += 1
                    yuv_frame.unref()
                    #self.get_logger().info(f'Publishing [{self.frameid}]')
                    #self.get_logger().info(f' // FPS: {round(1/(t-prev_t),2)})')
                
                prev_t = t
            except queue.Empty:
                pass
            except Exception as e:
                self.get_logger().info(f"Error processing frame: {e}")

    def flush_cb(self, stream):
        if stream["vdef_format"] != olympe.VDEF_I420:
            return True
        while not self.frame_queue.empty():
            self.frame_queue.get_nowait().unref()
        return True
    
    def on_press(self, key):
        if hasattr(key, 'char'):
            if key.char == 'p':
                self.Stop()


#---------------
# MAIN FUN
#----------------

def main(args=None):
    rclpy.init()
    af_pub = FramePub()
    af_pub.Connect()

    while rclpy.ok() and af_pub.Connected==True:
        rclpy.spin_once(af_pub)
    
    if af_pub.Connected:
        af_pub.Stop()

if __name__ == '__main__':
    main()
