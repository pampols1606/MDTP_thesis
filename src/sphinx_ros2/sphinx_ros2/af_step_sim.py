import rclpy
import rclpy.clock
from rclpy.node import Node
from anafi_msg.msg import VelData, PnPData

from sensor_msgs.msg import Image
from pynput.keyboard import Listener

import numpy as np
import os, olympe, time, csv, cv2
import pysphinx
import signal
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R

from pynput.keyboard import Key


from olympe.messages.ardrone3.Piloting import TakeOff, Landing, PCMD
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged

import warnings

warnings.simplefilter("ignore")
olympe.log.update_config({"loggers": {"olympe": {"level": "CRITICAL"}}})


class PoseDataSim(Node):
    def __init__(self):
        super().__init__('af_step_sim')

        #STOP FUNCTION HANDLER
        signal.signal(signal.SIGINT, self.Stop)

        self.listener = Listener(on_press=self.on_press)
        self.listener.start()

        self.sphinx = pysphinx.Sphinx(ip="127.0.0.1", port=8383)

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
        directory = '/home/ferran/Desktop/MATLAB/POSE_DATA'
        prefix = 'step_test_'
        suffix = '.csv'

        existing_files = [f for f in os.listdir(directory) if f.startswith(prefix) and f.endswith(suffix)]
        existing_numbers = [int(f[len(prefix):-len(suffix)]) for f in existing_files if f[len(prefix):-len(suffix)].isdigit()]
        next_number = max(existing_numbers, default=0) + 1

        csv_filename = f"{prefix}{next_number}{suffix}"

        csv_file_path = os.path.join(directory, csv_filename)
        self.csv_file = open(csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time [s]',
                                    'T_x [m]', 'T_y [m]', 'T_z [m]',
                                   'R_roll [deg]', 'R_pitch [deg]', 'R_yaw [deg]',
                                   'U_roll [%]','U_pitch [%]', 'U_yaw [%]', 'U_gaz [%]'])

    def Connect(self):
        self.get_logger().info('Connecting to Anafi drone...')

        self.DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")
        self.DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT")
        self.drone = olympe.Drone(self.DRONE_IP)

        for i in range(5):
            if self.Running:
                connected = self.drone.connect(retry=1)
                if connected:
                    self.Connected = True
                    self.get_logger().info('Conected to Anafi drone!')
                    time.sleep(0.2)
                    break
                else:
                    self.get_logger().info(f'Trying to connect ({i+1})')
                    time.sleep(2)
            
        if self.Connected:
            if self.DRONE_RTSP_PORT is not None:
                self.drone.streaming.server_addr = f"{self.DRONE_IP}:{self.DRONE_RTSP_PORT}"
                
        else:
            self.get_logger().info("Failed to connect.")
    
    def Stop(self, signal, frame):
        self.Connected = False
        self.Running = False

        cv2.destroyAllWindows()

        if self.Connected:
            self.Landing = True
            state = str(self.drone.get_state(FlyingStateChanged)['state'])
            if state != 'FlyingStateChanged_State.landed':
                self.get_logger().info(f'Landing...')
                self.drone(Landing()).wait().success()

            self.drone.disconnect()

        self.get_logger().info('Shutting down...\n')
        self.listener.stop()
        self.csv_file.close()
        time.sleep(0.1)

        self.destroy_node()
        rclpy.shutdown()
    
    def on_press(self, key):
        if key == Key.space:
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

        if key == Key.down:
            try:
                self.Landing = True
                self.Start_step=False
                time.sleep(0.2)
                self.drone(Landing()).wait().success()
                time.sleep(2)
                self.Landing = False
            except Exception as e:
                self.get_logger().warn(f"Failed to Land: {e}")

        if key == Key.up:
            try:
                self.TakingOff = True
                self.Start_step=False
                time.sleep(0.2)
                self.drone(TakeOff()).wait().success()
                time.sleep(3)
                self.TakingOff = False
            except  Exception as e:
                self.get_logger().warn(f"Failed to Take off: {e}")



    def main(self):
        init_t = ((self.get_clock().now().nanoseconds)/1000000000)

        pose = self.sphinx.get_drone_pose(machine_name="anafi")
        tras = [-pose[1], pose[2], -pose[0]]
        rot = [pose[3], pose[4], pose[5]]

        roll, pitch, yaw = rot[2], rot[0], (pose[5] - (np.arctan(tras[0]/tras[2])))*(180/(np.pi))

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
    af_step_sim = PoseDataSim()
    af_step_sim.Connect()

    while rclpy.ok() and af_step_sim.Connected==True:
        af_step_sim.main()


if __name__ == "__main__":
    main()