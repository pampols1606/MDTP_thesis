import rclpy
from rclpy.node import Node
from anafi_msg.msg import PnPData, ControlData, KpData
from vicon_receiver.msg import Position

import csv, os
import numpy as np

PoseSave = True
KpSave = False

class SaveData(Node):
    def __init__(self):
        super().__init__('af_save_data')

        self.targets_sub = self.create_subscription(PnPData, 'anafi/pnp', self.pnp_callback, 1)
        self.kp_sub = self.create_subscription(ControlData, 'anafi/control', self.control_callback, 1)
        self.anafi_sub = self.create_subscription(Position, 'vicon/anafi/anafi', self.anafi_callback, 1)
        self.bebop_sub = self.create_subscription(Position, 'vicon/bebop1/bebop1', self.bebop_callback, 1)
        self.kp_sub = self.create_subscription(KpData, 'anafi/bbox3D/kp', self.kp_callback, 1)

        self.create_timer(callback=self.main_bool, timer_period_sec=0.02)

        self.t_innit = self.get_clock().now().nanoseconds/10**9
        self.target_rpgy = np.array([0.0, 0.0, 2.0, 0.0])

        #File to save
        directory = './VICON_TEST_DATA'
        prefix = 'test_'
        suffix = '.csv'

        existing_files = [f for f in os.listdir(directory) if f.startswith(prefix) and f.endswith(suffix)]
        existing_numbers = [int(f[len(prefix):-len(suffix)]) for f in existing_files if f[len(prefix):-len(suffix)].isdigit()]
        next_number = max(existing_numbers, default=0) + 1

        csv_filename = f"{prefix}{next_number}{suffix}"

        csv_file_path = os.path.join(directory, csv_filename)
        self.csv_file = open(csv_file_path, 'w', newline='')

        if PoseSave:
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['Time [s]',
                                    'Detected_x [m]', 'Detected_y [m]', 'Detected_z [m]',
                                    'Target_x [m]', 'Target_y [m]', 'Target_z [m]',
                                    'Anafi_x [m]', 'Anafi_y [m]', 'Anafi_z [m]',
                                    'Bebop_x [m]', 'Bebop_y [m]', 'Bebop_z [m]',
                                    'Roll [%]', 'Pitch [%]', 'Yaw [%]', 'Gaz [%]'])
            
        if KpSave:
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['Time [s]',
                                    'Kp 1', 'Kp 2', 'Kp 3', 'Kp 4',
                                    'Kp 5', 'Kp 6', 'Kp 7', 'Kp 8',
                                    'Kp 9', 'Kp 10', 'Kp 11', 'Kp 12',
                                    'Kp 13', 'Kp 14', 'Kp 15', 'Kp 16'])

            self.Kp_received = False
        
        #Init
        self.kp_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.tx, self.ty, self.tz = 0, 0, 0

        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.pos_yaw = 0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.gaz = 0

        self.anafi_x = 0
        self.anafi_y = 0
        self.anafi_z = 0

        self.bebop_x = 0
        self.bebop_y = 0
        self.bebop_z = 0

    def kp_callback(self, msg):
        if len(msg.kp)!=0:
            self.kp_list = msg.kp
            self.Kp_received = True
        
    def pnp_callback(self, msg):
        self.tx, self.ty, self.tz = msg.tx, -msg.ty, msg.tz

        self.pos_x = (self.tx - self.target_rpgy[0])
        self.pos_y = (self.ty - self.target_rpgy[1])
        self.pos_z = (abs(self.tz) - self.target_rpgy[2])

    def control_callback(self, msg):

        self.roll = int(msg.roll)
        self.pitch = int(msg.pitch)
        self.yaw = int(msg.yaw)
        self.gaz = int(msg.gaz)
    
    def anafi_callback(self, msg):
        self.anafi_x = msg.x_trans
        self.anafi_y = msg.y_trans
        self.anafi_z = msg.z_trans

    def bebop_callback(self, msg):
        self.bebop_x = msg.x_trans
        self.bebop_y = msg.y_trans
        self.bebop_z = msg.z_trans


    def main_bool(self):
        
        try:
            time = (self.get_clock().now().nanoseconds/10**9) - self.t_innit

            if PoseSave:
                self.get_logger().info(f'Time: {time}')
                self.csv_writer.writerow([time,
                                        self.tx, self.ty, self.tz,
                                        self.pos_x, self.pos_y, self.pos_z,
                                        self.anafi_x, self.anafi_y, self.anafi_z,
                                        self.bebop_x, self.bebop_y, self.bebop_z,
                                        self.roll, self.pitch, self.yaw, self.gaz,])
            
            if KpSave and self.Kp_received:
                self.Kp_received = False
                self.csv_writer.writerow([time,
                                        self.kp_list[1],  self.kp_list[2],  self.kp_list[3],  self.kp_list[4],
                                        self.kp_list[5],  self.kp_list[6],  self.kp_list[7],  self.kp_list[8],
                                        self.kp_list[9],  self.kp_list[10],  self.kp_list[11],  self.kp_list[12],
                                        self.kp_list[13],  self.kp_list[14],  self.kp_list[15],  self.kp_list[16]])
        except Exception as e:
            self.get_logger().info(f"Error: {e}")
                  
    def Stop(self):
        
        self.csv_file.close()
        self.destroy_node()
        rclpy.shutdown()
    
def main():
    rclpy.init()
    af_save = SaveData()

    try:
        while rclpy.ok():
            rclpy.spin_once(af_save)
    
    except:
        af_save.Stop()


if __name__ == "__main__":
    main()
