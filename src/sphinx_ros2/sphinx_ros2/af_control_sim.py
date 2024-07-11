import rclpy
from rclpy.node import Node
from anafi_msg.msg import ControlData

from pynput.keyboard import Listener
import numpy as np

import signal
import pysphinx
import cv2

from pynput.keyboard import Key

class DroneControlSim(Node):
    def __init__(self):
        super().__init__("af_control_sim")

        #STOP FUNCTION HANDLER
        signal.signal(signal.SIGINT, self.Stop)

        #KEYBORD LISTENER
        self.listener = Listener(on_press=self.on_press)
        self.listener.start()

        self.declare_parameter('pid_popup', 'false')
        self.pid_popup = self.get_parameter('pid_popup').get_parameter_value().bool_value
        
        #MAIN TIMER Ts=0.05s
        self.Ts = 0.05
        self.control_timer = self.create_timer(self.Ts, self.main_timer)
        self.Running = True

        self.control_pub = self.create_publisher(ControlData, 'anafi/control', 1)
        self.sphinx = pysphinx.Sphinx(ip="127.0.0.1", port=8383)

        #CONTROL VARS
        self.target_pos = np.array([0., 0., 0., 0.])
        self.target_offset = np.array([-2.0, 0., 1.5, 0.])
        self.y_target = np.array([0., 0., 0., 0.])
        self.u_control = np.array([0., 0., 0., 0.])
        self.posYaw = 0.0
        self.max_u=[50,30,100,20]

        #PREVIOUS DATA
        self.y_feedback = np.array([0., 0., 0., 0.])
        self.error_prev = np.array([0., 0., 0., 0.])
        self.First_k = True

        #PID (ROLL PITCH YAW GAZ)
        n=2
        self.Kp = [n*6.2047, n*5.5103, 6.1384, 85.9881]
        self.Ki = [n*0.070493, n*0.063691, 0.37509, 1.3046]
        self.Kd = [n*43.2132, n*52.0765,  1.194,  3.8812]

        # self.Kp = [124.5006, 477.852, 80.2071, 11.7927]
        # self.Ki = [56.5379, 211.2172, 27.538, 14.8163]
        # self.Kd = [5.7744, 22.591, 4.5514, 0.56386]

        self.Kp_e = np.array([0.0, 0.0, 0.0, 0.0])
        self.Kd_e = np.array([0.0, 0.0, 0.0, 0.0])
        self.Ki_e = np.array([0.0, 0.0, 0.0, 0.0])

        #PID TRACKVBARS
        if self.pid_popup:

            cv2.namedWindow('PID Control', cv2.WINDOW_NORMAL)
            
            cv2.createTrackbar('Roll Kp', 'PID Control', 0, 10000, lambda x: None)
            cv2.createTrackbar('Roll Ki', 'PID Control', 0, 1000, lambda x: None)
            cv2.createTrackbar('Roll Kd', 'PID Control', 0, 100000, lambda x: None)

            cv2.createTrackbar('Pitch Kp', 'PID Control', 0, 10000, lambda x: None)
            cv2.createTrackbar('Pitch Ki', 'PID Control', 0, 1000, lambda x: None)
            cv2.createTrackbar('Pitch Kd', 'PID Control', 0, 100000, lambda x: None)

            cv2.createTrackbar('Yaw Kp', 'PID Control', 0, 50000, lambda x: None)
            cv2.createTrackbar('Yaw Ki', 'PID Control', 0, 50000, lambda x: None)
            cv2.createTrackbar('Yaw Kd', 'PID Control', 0, 50000, lambda x: None)

            cv2.createTrackbar('Gaz Kp', 'PID Control', 0, 200000, lambda x: None)
            cv2.createTrackbar('Gaz Ki', 'PID Control', 0, 50000, lambda x: None)
            cv2.createTrackbar('Gaz Kd', 'PID Control', 0, 50000, lambda x: None)

            # SET INICIAL PARAMETER
            cv2.setTrackbarPos('Roll Kp', 'PID Control', int(self.Kp[0]*1000))
            cv2.setTrackbarPos('Roll Ki', 'PID Control', int(self.Ki[0]*1000))
            cv2.setTrackbarPos('Roll Kd', 'PID Control', int(self.Kd[0]*1000))

            cv2.setTrackbarPos('Pitch Kp', 'PID Control', int(self.Kp[1]*1000))
            cv2.setTrackbarPos('Pitch Ki', 'PID Control', int(self.Ki[1]*1000))
            cv2.setTrackbarPos('Pitch Kd', 'PID Control', int(self.Kd[1]*1000))

            cv2.setTrackbarPos('Yaw Kp', 'PID Control', int(self.Kp[2]*1000))
            cv2.setTrackbarPos('Yaw Ki', 'PID Control', int(self.Ki[2]*1000))
            cv2.setTrackbarPos('Yaw Kd', 'PID Control', int(self.Kd[2]*1000))

            cv2.setTrackbarPos('Gaz Kp', 'PID Control', int(self.Kp[3]*1000))
            cv2.setTrackbarPos('Gaz Ki', 'PID Control', int(self.Ki[3]*1000))
            cv2.setTrackbarPos('Gaz Kd', 'PID Control', int(self.Kd[3]*1000))

        self.Pursuing_on = False


    def main_timer(self):

        #PID TRACKBARS
        if self.pid_popup:
            #PID TRACKBARS
            self.Kp[0] = cv2.getTrackbarPos('Roll Kp', 'PID Control')/1000.0
            self.Kp[1] = cv2.getTrackbarPos('Pitch Kp', 'PID Control')/1000.0
            self.Kp[2] = cv2.getTrackbarPos('Yaw Kp', 'PID Control')/1000.0
            self.Kp[3] = cv2.getTrackbarPos('Gaz Kp', 'PID Control')/1000.0

            self.Ki[0] = cv2.getTrackbarPos('Roll Ki', 'PID Control')/1000.0
            self.Ki[1] = cv2.getTrackbarPos('Pitch Ki', 'PID Control')/1000.0
            self.Ki[2] = cv2.getTrackbarPos('Yaw Ki', 'PID Control')/1000.0
            self.Ki[3] = cv2.getTrackbarPos('Gaz Ki', 'PID Control')/1000.0

            self.Kd[0] = cv2.getTrackbarPos('Roll Kd', 'PID Control')/1000.0
            self.Kd[1] = cv2.getTrackbarPos('Pitch Kd', 'PID Control')/1000.0
            self.Kd[2] = cv2.getTrackbarPos('Yaw Kd', 'PID Control')/1000.0
            self.Kd[3] = cv2.getTrackbarPos('Gaz Kd', 'PID Control')/1000.0

            cv2.waitKey(1)

        #CONTROL
        pose = self.sphinx.get_drone_pose(machine_name="anafi")
        
        yaw_rad = pose[5]
        yaw = yaw_rad*(180/np.pi)

        tx, ty, tz = -pose[0], pose[1], pose[2]
        

        if self.Pursuing_on:

            self.target_pos[0] = (self.target_offset[0]) # x
            self.target_pos[1] = (self.target_offset[1]) # y
            self.target_pos[2] = (self.target_offset[2]) # z                                       
            self.target_pos[3] = (self.target_offset[3]) # yaw

            
            if self.First_k:
                self.First_k = False
                self.y_prev = np.array([0.0, 0.0, 0.0, 0.0])
                self.error_prev = np.array([0.0, 0.0, 0.0, 0.0])
                self.Kp_e = np.array([0.0, 0.0, 0.0, 0.0])
                self.Kd_e = np.array([0.0, 0.0, 0.0, 0.0])
                self.Ki_e = np.array([0.0, 0.0, 0.0, 0.0])


            else:

                #System target and feedback states
                self.y_target[:] = self.target_pos[0], self.target_pos[1], self.target_pos[2], self.target_pos[3]
                self.y_feedback[:] = -tx, -ty, tz, -yaw
                
                #Absolut error
                error = self.y_target - self.y_feedback

                #Kp error
                self.Kp_e = error

                #Kd error
                self.Kd_e = (error - self.error_prev)/self.Ts

                #Ki error
                for i in range(0,4):
                    self.Ki_e[i] += error[i]*self.Ts

                    if  self.Ki_e[i] > 50.0:
                            self.Ki_e[i] = 50.0
                    elif  self.Ki_e[i] < -50.0:
                            self.Ki_e[i] = -50.0
                
                #Update error(k-1)
                self.error_prev = error

                #Control signal update
                K_roll = np.array([self.Kp[0], self.Ki[0], self.Kd[0]])
                K_pitch = np.array([self.Kp[1], self.Ki[1], self.Kd[1]])
                K_yaw = np.array([self.Kp[2], self.Ki[2], self.Kd[2]])
                K_gaz = np.array([self.Kp[3], self.Ki[3], self.Kd[3]])

                #X (pitch control)
                self.u_control[0] = K_pitch[0]*self.Kp_e[0] + K_pitch[1]*self.Ki_e[0] + K_pitch[2]*self.Kd_e[0]
                self.u_control[0] = max(-self.max_u[1], min(self.max_u[1], self.u_control[0]))
                
                #Y (roll control)
                self.u_control[1] = K_roll[0]*self.Kp_e[1] + K_roll[1]*self.Ki_e[1] + K_roll[2]*self.Kd_e[1] 
                self.u_control[1] = max(-self.max_u[0], min(self.max_u[0], self.u_control[1]))
                
                #Z (gaz control)
                self.u_control[2] = K_gaz[0]*self.Kp_e[2] +K_gaz[1]*self.Ki_e[2] + K_gaz[2]*self.Kd_e[2]
                self.u_control[2] = max(-self.max_u[3], min(self.max_u[3], self.u_control[2]))

                #Yaw (yaw control)
                self.u_control[3] = K_yaw[0]*self.Kp_e[3] + K_yaw[1]*self.Ki_e[3] + K_yaw[2]*self.Kd_e[3]
                self.u_control[3] = max(-self.max_u[2], min(self.max_u[2], self.u_control[3]))

                #Publisher
                u_pub = ControlData()
                u_pub.time = self.get_clock().now().nanoseconds/(10**9)
                u_pub.pitch, u_pub.roll, u_pub.gaz, u_pub.yaw =  self.u_control[0], self.u_control[1], self.u_control[2], self.u_control[3]
                self.control_pub.publish(u_pub)

                self.get_logger().info(f"POSE  : X: {tx} / Y: {ty} / Z: {tz} / YAW: {yaw}")
                self.get_logger().info(f"TARGET : X={round(self.y_target[0],3)} / Y={round(self.y_target[1],3)} / Z={round(self.y_target[2],3)} / YAW={round(self.y_target[3],3)}")
                self.get_logger().info(f"FEEDBACK : X={round(self.y_feedback[0],3)} / Y={round(self.y_feedback[1],3)} / Z={round(self.y_feedback[2],3)} / YAW={round(self.y_feedback[3],3)}")
                self.get_logger().info(f"ERROR: {error}")
                self.get_logger().info(f"CONTROL: Roll: {int(u_pub.roll)} / Pitch: {int(u_pub.pitch)} / Gaz: {int(u_pub.gaz)} / Yaw: {int(u_pub.yaw)} \n\n")
        
        else:

            u_pub = ControlData()
            u_pub.time = self.get_clock().now().nanoseconds/(10**9)
            u_pub.roll, u_pub.gaz, u_pub.pitch, u_pub.yaw =  0.0, 0.0, 0.0, 0.0
            self.control_pub.publish(u_pub)

            self.Kp_e = np.array([0.0, 0.0, 0.0, 0.0])
            self.Kd_e = np.array([0.0, 0.0, 0.0, 0.0])
            self.Ki_e = np.array([0.0, 0.0, 0.0, 0.0])

            #self.get_logger().info(f"POSE  : X: {tx} / Y: {ty} / Z: {tz} / YAW: {yaw}")

            self.First_k = True


    def Stop(self, signal, frame):
        self.Running = False
        cv2.destroyAllWindows()

        self.destroy_node()
        rclpy.shutdown()
    
    def on_press(self,key):
        if key == Key.space:
            if self.Pursuing_on:
                self.Pursuing_on = False
            else:
                self.Pursuing_on = True

def main(args=None):
    rclpy.init()
    af_control_sim = DroneControlSim()

    while rclpy.ok() and af_control_sim.Running:
        rclpy.spin_once(af_control_sim)

if __name__ == "__main__":
    main()