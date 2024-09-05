import ReadIMU as ReadIMU
import time
#from DNN import DNN
from DNN_torch import DNN
import datetime
import numpy as np
import csv
#import torch
#import torch.nn as nn

# class Network(nn.Module):
    # def __init__(self):
        # super(Network,self).__init__()
        # self.fc1 = nn.Linear(18,128)
        # self.fc2 = nn.Linear(128,64)
        # self.fc3 = nn.Linear(64,2)
    
    # def forward(self,x):
        # x = torch.relu(self.fc1(x))
        # x = torch.relu(self.fc2(x))
        # x = self.fc3(x)
        # return x

# network = Network()
# input_data = torch.rand(1,18)

# Initialize IMU and DNN
#ComPort = '/dev/ttyUSB0'
ComPort = '/dev/serial0'
imu = ReadIMU.READIMU(ComPort)
start = time.time()
dnn = DNN(18, 128, 64, 2)
now = 0
t_pr1 = 0
t_pr2 = 0
t_pr3 = 0
pk = 0
counter = 0

date = time.localtime(time.time())
date_year = date.tm_year
date_month = date.tm_mon
date_day = date.tm_mday
date_hour = date.tm_hour
date_minute = date.tm_min
date_second = date.tm_sec

output = np.array([])

# Create filename with format {Year}{Month}{Day}-{Hour}{Minute}{Second}.csv
csv_filename = f"{date_year:04}{date_month:02}{date_day:02}-{date_hour:02}{date_minute:02}{date_second:02}.csv"
with open(csv_filename, 'a', newline='') as csvfile:
    fieldnames = ['L_IMU_Ang', 'R_IMU_Ang', 'L_IMU_Vel', 'R_IMU_Vel', 'L_Cmd', 'R_Cmd', 'Peak', 'Time']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    # Write the header only if the file is empty
    csvfile.seek(0, 2)
    if csvfile.tell() == 0:
        writer.writeheader()

    while True:
        now = (time.time() - start)
        imu.read()
        imu.decode() 
        print("count :", counter) 

        # if counter % 1000 == 0:
        #     imu.Serial_IMU.reset_input_buffer()
        #     imu.Serial_IMU.reset_output_buffer() 
        #     # print("----------------------------------------")
        #     # print("----------------------------------------")
        #     # print("----------------------------------------")
        #     # print("----------------------------------------") 
        #     # print("----------------------------------------")

        counter = counter + 1
        L_IMU_angle = imu.XIMUL
        R_IMU_angle = imu.XIMUR
        L_IMU_vel = imu.XVIMUL
        R_IMU_vel = imu.XVIMUR
        print(f"Time after reading IMU = {now:^8.3f}")

        
        # L_Cmd = 0
        # R_Cmd = 0
        if (now - t_pr3 > 3):  # Time to reset peak torque printed to terminal
            t_pr3 = now
            pk = 0

        if (now - t_pr1 > 0.001):
            t_pr1 = now
            kp = 10
            kd = 400
            print(f"Time when running NN = {now:^8.3f}")
            dnn.generate_assistance(L_IMU_angle, R_IMU_angle, L_IMU_vel, R_IMU_vel, kp, kd)
            # output = network(input_data)

            kcontrol = 1  # 1.5 for running, 2 for climbing

            L_Cmd = -dnn.hip_torque_L * 1.0 * kcontrol
            R_Cmd = dnn.hip_torque_R * 1.0 * kcontrol

            if (L_Cmd > pk or R_Cmd > pk):
                if (R_Cmd > L_Cmd):
                    pk = R_Cmd
                if (L_Cmd > R_Cmd):
                    pk = L_Cmd

            B1_int16 = int(imu.ToUint(L_Cmd/20.0, -20, 20, 16))
            B2_int16 = int(imu.ToUint(R_Cmd/20.0, -20, 20, 16))

            b1 = (B1_int16 >> 8 & 0x00ff)
            b2 = (B1_int16 & 0x00FF)
            b3 = (B2_int16 >> 8 & 0x00ff)
            b4 = (B2_int16 & 0x00FF)

            imu.send(b1, b2, b3, b4)

            data = {
                'L_IMU_Ang': L_IMU_angle,
                'R_IMU_Ang': R_IMU_angle,
                'L_IMU_Vel': L_IMU_vel,
                'R_IMU_Vel': R_IMU_vel,
                'L_Cmd': L_Cmd,
                'R_Cmd': R_Cmd,
                'Peak': pk,
                'Time': now
            }
            writer.writerow(data)
            csvfile.flush()  # Ensure data is written to file 
            print(f"| now: {now:^8.3f} | L_IMU_Ang: {L_IMU_angle:^8.3f} | R_IMU_Ang: {R_IMU_angle:^8.3f} | L_IMU_Vel: {L_IMU_vel:^8.3f} | R_IMU_Vel: {R_IMU_vel:^8.3f} | L_Cmd: {L_Cmd:^8.3f} | R_Cmd: {R_Cmd:^8.3f} | Peak: {pk:^8.3f} |")
        # if (now - t_pr2 > 0.001):
            # t_pr2 = now
            # print(f"| now: {now:^8.3f} | L_IMU_Ang: {L_IMU_angle:^8.3f} | R_IMU_Ang: {R_IMU_angle:^8.3f} | L_IMU_Vel: {L_IMU_vel:^8.3f} | R_IMU_Vel: {R_IMU_vel:^8.3f} | L_Cmd: {L_Cmd:^8.3f} | R_Cmd: {R_Cmd:^8.3f} | Peak: {pk:^8.3f} |")
            
            # # Save the data to the CSV file
            # data = {
                # 'L_IMU_Ang': L_IMU_angle,
                # 'R_IMU_Ang': R_IMU_angle,
                # 'L_IMU_Vel': L_IMU_vel,
                # 'R_IMU_Vel': R_IMU_vel,
                # 'L_Cmd': L_Cmd,
                # 'R_Cmd': R_Cmd,
                # 'Peak': pk,
                # 'Time': now
            # }
            # writer.writerow(data)
            # csvfile.flush()  # Ensure data is written to file 
