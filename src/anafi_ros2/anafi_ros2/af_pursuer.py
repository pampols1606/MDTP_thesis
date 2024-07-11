import rclpy
import rclpy.clock
import rclpy.time
import warnings
import numpy as np
import cv2
import os
import olympe
import time
import queue

from rclpy.node import Node
from sensor_msgs.msg import Image
from anafi_msg.msg import ControlData, StateData
from cv_bridge import CvBridge
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, PCMD
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, SpeedChanged, AttitudeChanged
from olympe.messages.ardrone3.PilotingSettingsState import MaxTiltChanged
from olympe.messages.ardrone3.SpeedSettingsState import MaxRotationSpeedChanged, MaxVerticalSpeedChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.SpeedSettings import MaxVerticalSpeed, MaxRotationSpeed
from olympe.messages.ardrone3.SpeedSettingsState import MaxRotationSpeedChanged, MaxVerticalSpeedChanged
from pynput.keyboard import Listener, Key
from scipy.spatial.transform import Rotation

#IGNORE WARNINGS
warnings.simplefilter("ignore")
olympe.log.update_config({"loggers": {"olympe": {"level": "CRITICAL"}}})

class DronePursuer(Node):
    def __init__(self):
        super().__init__('af_pursuer')

        #LAUNCH PARAMETERS  
        self.declare_parameter('sim', 'false')
        self.sim = self.get_parameter('sim').get_parameter_value().bool_value

        #SET MAIN TIMER
        self.freq = 40 #Hz
        self.processing_timer = self.create_timer(callback=self.main_bool, timer_period_sec= 1/self.freq)
        
        #PUBLISHERS / SUBSCRIBERS
        self.control_sub = self.create_subscription(ControlData, 'anafi/control', self.control_callback, 1)
        self.image_pub = self.create_publisher(Image, 'anafi/frames', 100)
        self.state_pub = self.create_publisher(StateData, 'anafi/state', 1)

        #KEYBORD LISTENER
        self.listener = Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        #CV BRIDGE
        self.bridge = CvBridge()
        self.frame_queue = queue.LifoQueue()

        self.cv2_cvt_color_flag = {
                    olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
                    olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,}
        
        #VARS
        self.Running = True
        self.Connected = False
        self.Pursuing_on = False
        self.frame_id_pub = 0

        #MANUAL MODE
        self.roll_manual = 0
        self.pitch_manual = 0
        self.yaw_manual = 0
        self.gaz_manual = 0

        #AUTOMATIC MODE
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.gaz = 0
        self.t = 0
        self.target_id = 0

        #VELOCITIES

        self.world_vel = np.array([0., 0., 0])  # This velocity is velocity in map frame!
        self.body_vel = np.array([0., 0., 0.])  # This velocity is velocity from the drone's frame!
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0

    def control_callback(self, msg):
        self.t = msg.time
        self.target_id = msg.target_id
        self.roll = int(msg.roll)
        self.pitch = int(msg.pitch)
        self.yaw = int(msg.yaw)
        self.gaz = int(msg.gaz)
    
    def yuv_frame_cb(self, yuv_frame):
        
        try:
            yuv_frame.ref()
            self.frame_queue.put_nowait(yuv_frame)
        except Exception as e:
            self.get_logger().info(f"Error handling media removal: {e}")
    

    def yuv_frame_processing(self):

        try:
            yuv_frame = self.frame_queue.get(timeout=0.1)

            if yuv_frame is not None:
                x = yuv_frame.as_ndarray()
                cv2frame = cv2.cvtColor(x, self.cv2_cvt_color_flag[yuv_frame.format()])
                msg = self.bridge.cv2_to_imgmsg(cv2frame, "bgr8")
                msg.header.frame_id = str(self.frame_id_pub)
                msg.header.stamp.sec = int(self.get_clock().now().nanoseconds/10**9)
                self.image_pub.publish(msg)
                self.frame_id_pub += 1
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


    #CONNECTING FUNCTION
    def Connect(self):
        self.get_logger().info('Connecting to Anafi drone...')
        
        if self.sim:
            self.DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")
        else:
            self.DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")

        self.DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT")
        self.drone = olympe.Drone(self.DRONE_IP)

        for i in range(5):
            if self.Running:
                connected= self.drone.connect(retry=1)
                if connected:
                    self.Connected = True
                    self.get_logger().info('Conected to Anafi drone!')
                    break
                else:
                    self.get_logger().info(f'Trying to connect ({i+1})')
                    time.sleep(2)
            
        if self.Connected:
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

            if self.DRONE_RTSP_PORT is not None:
                self.drone.streaming.server_addr = f"{self.DRONE_IP}:{self.DRONE_RTSP_PORT}"
            self.drone.streaming.set_callbacks(raw_cb=self.yuv_frame_cb, flush_raw_cb=self.flush_cb,)
            self.drone.streaming.start()

        else:
            self.get_logger().info("Failed to connect.")
    

    #STOP FUNCTION
    def Stop(self):
        self.Running = False
        self.Pursuing_on = False
        self.listener.stop()

        if self.Connected:
            self.drone(PCMD(0,0,0,0,0,timestampAndSeqNum=0,))

            FlyingState = str(self.drone.get_state(FlyingStateChanged)['state'])
            if FlyingState != 'FlyingStateChanged_State.landed':
                self.drone(Landing()).wait().success()
            self.drone.streaming.stop()
            self.drone.disconnect()

        self.destroy_node()
        rclpy.shutdown()
    
    #DRONE STATE PUBLISHER
    def state_publishing(self):

        state_msg = StateData()
        state_msg.time = (self.get_clock().now().nanoseconds)/10**9
        
        speed_dict = self.drone.get_state(SpeedChanged)
        self.world_vel[0] = speed_dict['speedX']
        self.world_vel[1] = -speed_dict['speedY']
        self.world_vel[2] = -speed_dict['speedZ']

        orientation_dict = self.drone.get_state(AttitudeChanged)
        roll = orientation_dict['roll']
        pitch = -orientation_dict['pitch']
        yaw = -orientation_dict['yaw']

        true_orient = Rotation.from_euler('ZYX', [yaw, 0, 0], degrees=False)

        self.body_vel = true_orient.apply(self.world_vel, inverse=True)

        state_msg.speed_x = self.body_vel[0] # m/s
        state_msg.speed_y = self.body_vel[1] # m/s
        state_msg.speed_z = self.body_vel[2] # m/s
        state_msg.roll = roll*(180/np.pi) # deg
        state_msg.pitch = -pitch*(180/np.pi) # deg
        state_msg.yaw = -yaw*(180/np.pi) # deg

        self.state_pub.publish(state_msg)

    #DRONE MANUAL / AUTOMATIC CONTROL
    def drone_control(self):

        if self.Pursuing_on and self.target_id!=0:
            self.drone(PCMD(1,self.roll, self.pitch, self.yaw, self.gaz,timestampAndSeqNum=0,))
                                      
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
    
    #MAIN TIMER BOOL
    def main_bool(self):
        try:
            if self.Running and self.Connected:
                self.yuv_frame_processing()
                self.state_publishing()
                self.drone_control()
        except Exception as e:
            self.get_logger().info(f"Error: {e}")
    

    #KEYBOARD PRESS KEY FUNCTION
    def on_press(self, key):

        if key == Key.space:
            if self.Pursuing_on:
                self.get_logger().info(f"--MANUAL MODE -- \n\n")
                self.Pursuing_on = False
                self.drone(PCMD(0,0,0,0,0,timestampAndSeqNum=0,))
            else:
                self.get_logger().info(f"--PURSUING MODE-- \n\n")
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
                    self.gaz_manual = 25
                elif key.char == 'f':
                    self.gaz_manual = -25
                elif key.char == 'c':
                    self.yaw_manual = 100
                elif key.char == 'x':
                    self.yaw_manual = -100
        
        if key == Key.esc:
            self.Stop()

    #KEYBOARD RELEASE KEY FUNCTION
    def on_release(self,key):
        if hasattr(key, 'char') and key.char in ['w', 's']:
            self.pitch_manual = 0
        if hasattr(key, 'char') and key.char in ['a', 'd']:
            self.roll_manual = 0
        if hasattr(key, 'char') and key.char in ['r', 'f']:
            self.gaz_manual = 0
        if hasattr(key, 'char') and key.char in ['x', 'c']:
            self.yaw_manual = 0


def main():
    rclpy.init()
    af_pursuer = DronePursuer()
    af_pursuer.Connect()

    try:
        while rclpy.ok() and af_pursuer.Connected==True:
            rclpy.spin_once(af_pursuer, timeout_sec=0.1)
    except:
        af_pursuer.Stop()

if __name__ == "__main__":
    main()