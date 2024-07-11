import rclpy
import rclpy.clock
from rclpy.node import Node
from vicon_receiver.msg import Position

from sensor_msgs.msg import Image
from pynput.keyboard import Listener, Key

import numpy as np
import os, olympe, time, csv, cv2
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R


from olympe.messages.ardrone3.Piloting import TakeOff, Landing, PCMD

import warnings

warnings.simplefilter("ignore")
olympe.log.update_config({"loggers": {"olympe": {"level": "CRITICAL"}}})


class ViconData(Node):
    def __init__(self):
        super().__init__('af_vicon_step')
        self.vicon_sub = self.create_subscription(Position, "vicon/anafi/anafi", self.vicon_callback, 1)

        self.listener = Listener(on_press=self.on_press)
        self.listener.start()

        self.bridge = CvBridge()
        self.Running = True
        self.Connected = False
        self.Stopped = False
        self.step_type = [0, 0, 0 ,0] # roll pitch yaw gaz

        self.t1 = 0
        self.t2 = 0
        self.Start_step = False
        self.TakingOff = False
        self.Landing = False
        self.step_time = 0 # in sec
        cv2.namedWindow('Control', cv2.WINDOW_NORMAL)
        cv2.createTrackbar('Time', 'Control', 0, 4000, lambda x: None)
        cv2.createTrackbar('Roll', 'Control', 0, 70, lambda x: None)
        cv2.createTrackbar('Pitch', 'Control', 0, 70, lambda x: None)
        cv2.createTrackbar('Yaw', 'Control', 0, 100, lambda x: None)
        cv2.createTrackbar('Gaz', 'Control', 0, 70, lambda x: None)

        # Abre el archivo CSV para escritura
        self.csv_file = open('vicon_data.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time [s]',
                                    'T_x [m]', 'T_y [m]', 'T_z [m]',
                                   'R_roll [deg]', 'R_pitch [deg]', 'R_yaw [deg]',
                                    'U_roll [%]', 'U_pitch [%]', 'U_yaw [%]', 'U_gaz [%]'])

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
            
        else:
            self.get_logger().info("Failed to connect.")
    
    def Stop(self):
        
        self.Running = False
        cv2.destroyAllWindows()
        if self.Connected:
            self.drone.disconnect()

        self.get_logger().info('Shutting down...\n')
        self.listener.stop()
        self.Connected = False
        self.csv_file.close()
        time.sleep(0.2)
        self.destroy_node()
        rclpy.shutdown()
    
    def on_press(self, key):
        if key == Key.esc:
            self.get_logger().info('Stopping...\n')
            self.Landing = True
            time.sleep(0.1)
            self.drone(Landing()).wait().success()
            self.Connected = False
            time.sleep(0.5)
            self.Stop()

        elif key== Key.space:
            if self.Start_step:
                self.Start_step = False
                time.sleep(0.2)
                self.drone(PCMD(0,0,0,0,0, timestampAndSeqNum=0))
                self.t1 = 0
                self.t2 = 0
            else:
                self.t1 = 0
                self.t2 = (self.get_clock().now().nanoseconds)/1000000000
                self.Start_step = True

        elif key== Key.down:
            try:
                self.Landing = True
                self.Start_step=False
                time.sleep(0.2)
                self.drone(Landing()).wait().success()
                time.sleep(2)
                self.Landing = False
            except Exception as e:
                self.get_logger().warn(f"Failed to Land: {e}")

        elif key == Key.up:
            try:
                self.TakingOff = True
                self.Start_step=False
                time.sleep(0.2)
                self.drone(TakeOff()).wait().success()
                time.sleep(3)
                self.TakingOff = False
            except  Exception as e:
                self.get_logger().warn(f"Failed to Take off: {e}")



    def vicon_callback(self, msg):
        init_t = ((self.get_clock().now().nanoseconds)/1000000000)

        tras = [msg.x_trans, msg.y_trans, msg.z_trans]
        rot = [msg.x_rot, msg.y_rot, msg.z_rot, msg.w]

        quaternion = (rot[0], rot[1], rot[2], rot[3])
        rotation = R.from_quat(quaternion)

        roll, pitch, yaw = rotation.as_euler('xyz', degrees=True)

        rot_z = R.from_euler('z', yaw, degrees=True).inv()
        tras = rot_z.apply(tras)

        cv2.waitKey(1)

        self.step_time = cv2.getTrackbarPos('Time', 'Control')/1000
        self.step_type[0] = cv2.getTrackbarPos('Roll', 'Control')
        self.step_type[1] = cv2.getTrackbarPos('Pitch', 'Control')
        self.step_type[2] = cv2.getTrackbarPos('Yaw', 'Control')
        self.step_type[3] = cv2.getTrackbarPos('Gaz', 'Control')

        self.t1 = ((self.get_clock().now().nanoseconds)/1000000000) - self.t2
        u=[0,0,0,0]

        if not self.Landing and not self.TakingOff:
            if self.Connected and self.Start_step:
                if self.t1< 1*self.step_time:
                    u = [0, 0, 0, 0]
                    print(f"Step 1 // time = {self.step_time}// u = {u}")

                elif self.t1>= 1*self.step_time and self.t1< 2*self.step_time:
                    u = [self.step_type[0], self.step_type[1], self.step_type[2], self.step_type[3]]
                    print(f"Step 2 // time = {self.step_time}// u = {u}")

                elif self.t1>= 2*self.step_time and self.t1< 3*self.step_time:
                    u = [0, 0, 0, 0]
                    print(f"Step 3 // time = {self.step_time}// u = {u}")

                elif self.t1>= 3*self.step_time and self.t1< 4*self.step_time:
                    u = [-self.step_type[0], -self.step_type[1], -self.step_type[2], -self.step_type[3]]
                    print(f"Step 4 // time = {self.step_time}// u = {u}")

                if self.t1>= 4*self.step_time:
                    self.t1= 0
                    self.t2 = (self.get_clock().now().nanoseconds)/1000000000
                    print("finish")

                self.drone(PCMD(1,u[0],u[1],u[2],u[3], timestampAndSeqNum=0))
                
            else:
                print("paused")
        
            #Write data to csv file
            self.csv_writer.writerow([init_t,
                                    tras[0],tras[1],tras[2], 
                                    roll, pitch, yaw,
                                    u[0], u[1], u[2], u[3]])
        
        
        end_t =((self.get_clock().now().nanoseconds)/1000000000)
        time.sleep(max(0.05-(end_t-init_t), 0))

def main():
    rclpy.init()
    af_vicon_step = ViconData()
    af_vicon_step.Connect()

    while rclpy.ok() and af_vicon_step.Connected==True:
        rclpy.spin_once(af_vicon_step)
    
    if af_vicon_step.Connected:
        af_vicon_step.Stop()

if __name__ == "__main__":
    main()