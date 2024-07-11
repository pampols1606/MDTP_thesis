import rclpy
import rclpy.clock
from rclpy.node import Node
from anafi_msg.msg import ControlData
import rclpy.time
from sensor_msgs.msg import Image
from pynput.keyboard import Listener, Key

import numpy as np
import cv2, os, olympe, time, threading, queue
from cv_bridge import CvBridge
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, PCMD
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettingsState import MaxTiltChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.SpeedSettings import MaxPitchRollRotationSpeed, MaxVerticalSpeed, MaxRotationSpeed
from olympe.messages.ardrone3.SpeedSettingsState import MaxPitchRollRotationSpeedChanged, MaxRotationSpeedChanged, MaxVerticalSpeedChanged

import warnings

import warnings
import signal
import os

warnings.simplefilter("ignore")
olympe.log.update_config({"loggers": {"olympe": {"level": "CRITICAL"}}})

class DronePursuerSim(Node):
    def __init__(self):
        super().__init__('af_pursuer_sim')

        # STOP FUNCTION HANDLER
        signal.signal(signal.SIGINT, self.Stop)

        self.vel_sub = self.create_subscription(ControlData,'anafi/control', self.vel_callback, 1)
        self.image_pub = self.create_publisher(Image, 'anafi/frames', 5)

        self.listener = Listener(on_press=self.on_press, on_release=self.on_release)
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
        self.Pursuing_on = False
        self.frameid = 0
        self.i = 0

        #MANUAL CTRL
        self.roll_manual = 0
        self.pitch_manual = 0
        self.yaw_manual = 0
        self.gaz_manual = 0

    def Connect(self):
        self.get_logger().info('Connecting to Anafi drone...')

        self.DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")
        self.DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT")
        self.drone = olympe.Drone(self.DRONE_IP)

        for i in range(5):
            if self.Running:
                connected = self.drone.connect(retry=1)
                if connected:
                    #Setting max values
                    maxTiltAction = self.drone(MaxTilt(10)).wait()
                    maxRotationSpeedAction = self.drone(MaxRotationSpeed(56)).wait()
                    maxVerticalSpeedAction = self.drone(MaxVerticalSpeed(1)).wait()

                    max_tilt = str(self.drone.get_state(MaxTiltChanged)['current'])
                    speed_yaw_max = self.drone.get_state(MaxRotationSpeedChanged)['current']
                    speed_gaz_max = self.drone.get_state(MaxVerticalSpeedChanged)['current']

                    self.get_logger().info(f'Pitch/Roll tilt limit: {str(max_tilt)}')
                    self.get_logger().info(f'Yaw speed limit: {str(speed_yaw_max)}')
                    self.get_logger().info(f'Gaz speed limit: {str(speed_gaz_max)}')

                    time.sleep(0.2)
                    self.Connected = True
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
    
    def Stop(self, signal, frame):
        self.Running = False
        self.Pursuing_on = False
    
        time.sleep(0.05)
        self.drone(PCMD(0,0,0,0,0,timestampAndSeqNum=0,))

        self.get_logger().info('Shutting down pursuer...')

        if self.Connected:
            state = str(self.drone.get_state(FlyingStateChanged)['state'])
            if state != 'FlyingStateChanged_State.landed':
                self.get_logger().info(f'Landing...')
                self.drone(Landing()).wait().success()

            self.drone.streaming.stop()
            self.drone.disconnect()
            self.Connected = False

        self.listener.stop()

        self.destroy_node()
        rclpy.shutdown()


    def yuv_frame_cb(self, yuv_frame):
        
        try:
            yuv_frame.ref()
            self.frame_queue.put_nowait(yuv_frame)

        except Exception as e:
            self.get_logger().info(f"Error handling media removal: {e}")


    def yuv_frame_processing(self):
        while self.Connected:
            try:
                yuv_frame = self.frame_queue.get(timeout=0.1)
                
                if yuv_frame is not None:
                    x = yuv_frame.as_ndarray()
                    cv2frame = cv2.cvtColor(x, self.cv2_cvt_color_flag[yuv_frame.format()])
                    msg = self.bridge.cv2_to_imgmsg(cv2frame, "bgr8")
                    msg.header.frame_id = str(self.frameid)
                    self.image_pub.publish(msg)
                    self.frameid += 1
                    yuv_frame.unref()
                
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

    def vel_callback(self,msg):
        roll = int(msg.roll)
        pitch = int(msg.pitch)
        yaw = int(msg.yaw)
        gaz = int(msg.gaz)

        if self.Pursuing_on:
            self.i = self.i + 1

            if self.i>5:            
                self.drone(PCMD(1,roll,pitch,yaw,gaz,timestampAndSeqNum=0,))

        else:
            if (self.roll_manual or self.pitch_manual or self.yaw_manual or self.gaz_manual) != 0:
                self.drone(PCMD(1,
                                self.roll_manual,
                                self.pitch_manual,
                                self.yaw_manual,
                                self.gaz_manual,
                                timestampAndSeqNum=0,))
            else:
                self.drone(PCMD(0,0,0,0,0,timestampAndSeqNum=0,))
            self.i = 0

    def on_press(self, key):

        if key == Key.space:
            if self.Pursuing_on:
                self.get_logger().info(f"Stopping pursuing.")
                self.Pursuing_on = False
                self.drone(PCMD(0,0,0,0,0,timestampAndSeqNum=0,))
            else:
                self.get_logger().info(f"Starting pursuing.")
                self.Pursuing_on = True

        elif key == Key.down:
            time.sleep(0.1)
            self.Pursuing_on = False
            try:
                self.drone(Landing())
            except Exception as e:
                self.get_logger().info(f"Failed to Land.")
            time.sleep(0.5)
        
        elif key == Key.up:
            time.sleep(0.1)
            try:
                self.drone(TakeOff())
            except Exception as e:
                self.get_logger().info(f"Failed to Take Off.")
            time.sleep(0.5)

        elif hasattr(key, 'char') and key.char and not self.Pursuing_on:
                if key.char == 'w':
                    self.pitch_manual = 50
                elif key.char == 's':
                    self.pitch_manual = -50
                elif key.char == 'd':
                    self.roll_manual = 50
                elif key.char == 'a':
                    self.roll_manual = -50
                elif key.char == 'r':
                    self.gaz_manual = 50
                elif key.char == 'f':
                    self.gaz_manual = -50
                elif key.char == 'c':
                    self.yaw_manual = 50
                elif key.char == 'x':
                    self.yaw_manual = -50

        elif key == Key.esc:
            self.Stop(1,1)
        
    def on_release(self,key):
        if hasattr(key, 'char') and key.char in ['w', 's']:
            self.pitch_manual = 0
        if hasattr(key, 'char') and key.char in ['a', 'd']:
            self.roll_manual = 0
        if hasattr(key, 'char') and key.char in ['r', 'f']:
            self.gaz_manual = 0
        if hasattr(key, 'char') and key.char in ['x', 'c']:
            self.yaw_manual = 0


def main(args=None):
    rclpy.init(args=args)
    af_pursuer_sim = DronePursuerSim()
    af_pursuer_sim.Connect()

    while rclpy.ok() and af_pursuer_sim.Running:
            rclpy.spin_once(af_pursuer_sim, timeout_sec=0.01)


if __name__ == "__main__":
    main()