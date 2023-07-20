import asyncio
import threading
import time
from datetime import datetime
from pathlib import Path

import serial_asyncio
import pandas as pd
import json
import csv
from imucal import ferraris_regions_from_section_list, TurntableCalibration, ferraris_regions_from_interactive_plot
from imucal.management import save_calibration_info
import matplotlib.pyplot as plt
from numpy.linalg import norm
import re

import gui


class WMD3Protocol(asyncio.Protocol):
    op_number = 0
    received_message = b''
    # X+: 1, X-: 2, Y+: 3, Y-: 2, Z+: 0, Z-: 4, 5 - отсечка,
    # X+ rotate: 6, 7 - отсечка, Y+ rotate: 8, 0 - отсечка, Z+ rotate: 9.
    commandsToWMD3 = [b"home0\n",  # 0
                      b"0deg00090.000-00000.000-00000.000-00000.000-040.0\n",  # 1
                      b"0deg00000.000-00180.000-00000.000-00000.000-040.0\n",  # 2
                      b"0deg00000.000--0090.000-00000.000-00000.000-040.0\n",  # 3
                      b"0deg00180.000-00000.000-00000.000-00000.000-040.0\n",  # 4
                      b"0deg00000.000-00090.000-00000.000-00000.000-040.0\n",  # 5
                      b"0deg00720.000-00000.000-00000.000-00000.000-040.0\n",  # 6
                      b"0deg00000.000--0090.000-00000.000-00000.000-040.0\n",  # 7
                      b"0deg-0720.000-00000.000-00000.000-00000.000-040.0\n",  # 8
                      b"0deg00000.000-00720.000-00000.000-00000.000-040.0\n"]  # 9
    message_number = 0

    def connection_made(self, transport):
        self.transport = transport
        print('port opened', transport)
        transport.serial.rts = False
        self.op_number = 0
        self.message_number = 0
        self.send_message(0)

    def data_received(self, data):
        self.received_message += data
        # print('data received', repr(data))
        print('received_message', repr(self.received_message))
        if self.received_message == b'degRECV' or self.received_message == b'degRECVdegRECV':
            self.received_message = b''
            if self.message_number in [7, 9, 10]:
                check_boundaries(self.message_number, True)
        elif len(self.received_message) == 42:
            self.received_message = b''
            self.check_operation(self.message_number)
        if b'\n' in data:
            self.transport.close()

    def connection_lost(self, exc):
        print('port closed')
        self.transport.loop.stop()

    def send_message(self, message_num):
        self.message_number = message_num
        print("Sent message:", f"Message number: {message_num}; Operation: {self.get_op_number(message_num)}; "
                               f"Operation str: {self.commandsToWMD3[self.get_op_number(message_num)]}")
        self.transport.write(self.commandsToWMD3[self.get_op_number(message_num)])

    def get_op_number(self, message_num):
        if message_num in [4]:
            return 0
        elif message_num == 0:
            return 1
        elif message_num in [1, 3]:
            return 2
        elif message_num == 2:
            return 3
        elif message_num == 5:
            return 4
        elif message_num == 6:
            return 5
        elif message_num == 7:
            return 6
        elif message_num == 8:
            return 7
        elif message_num == 9:
            return 8
        elif message_num == 10:
            return 9

    def check_operation(self, message_num):
        if 0 <= message_num <= 5:
            check_boundaries(message_num, True)
            time.sleep(5)
            check_boundaries(message_num, False)
            self.send_message(message_num + 1)
        elif message_num in [6, 8]:
            self.send_message(message_num + 1)
        else:
            check_boundaries(message_num, False)
            if message_num == 10:
                self.transport.close()
            else:
                self.send_message(message_num + 1)


class ImuSensorProtocol(asyncio.Protocol):
    received_message = b''

    def connection_made(self, transport):
        self.transport = transport

    def data_received(self, data):
        self.received_message += data
        # print('imu received', repr(self.received_message), repr(self.received_message)[-2])
        if repr(self.received_message)[-2] == 'n':
            # print('data received', repr(self.received_message))
            imu_raw = self.received_message.decode('utf-8').split('\r\n')[0].split(',')
            imu_list = [csv_row_count, imu_raw[3], imu_raw[4], imu_raw[5], imu_raw[0], imu_raw[1], imu_raw[2]]
            if csvFile.closed:
                self.transport.close()
                self.transport.loop.stop()
            else:
                process_imu_data(imu_list)
            self.received_message = b''

        # self.pause_reading()

    def pause_reading(self):
        self.transport.pause_reading()

    def resume_reading(self):
        self.transport.resume_reading()


csv_row_count = 0
csvFile = open('imu_raw_data.csv', 'w', encoding='UTF8', newline='')
csv_writer = csv.writer(csvFile)
csv_writer.writerow(["n_samples", "gyr_x", "gyr_y", "gyr_z", "acc_x", "acc_y", "acc_z"])

json_dictionary = {}
jsonFile = open('ferraris_session.json', 'w')


def process_imu_data(imu_list):
    global csv_row_count
    # print(imu_list)
    csv_writer.writerow(imu_list)
    csv_row_count += 1


def check_boundaries(op, is_write):
    if is_write is True:
        if op == 0:
            json_dictionary["x_p"] = {"start": csv_row_count + 1}
        elif op == 1:
            json_dictionary["x_a"] = {"start": csv_row_count + 1}
        elif op == 2:
            json_dictionary["y_p"] = {"start": csv_row_count + 1}
        elif op == 3:
            json_dictionary["y_a"] = {"start": csv_row_count + 1}
        elif op == 4:
            json_dictionary["z_a"] = {"start": csv_row_count + 1}
        elif op == 5:
            json_dictionary["z_p"] = {"start": csv_row_count + 1}
        elif op == 7:
            json_dictionary["x_rot"] = {"start": csv_row_count + 15}
        elif op == 9:
            json_dictionary["y_rot"] = {"start": csv_row_count + 15}
        elif op == 10:
            json_dictionary["z_rot"] = {"start": csv_row_count + 15}
    else:
        if op == 0:
            json_dictionary["x_p"]["end"] = csv_row_count
        elif op == 1:
            json_dictionary["x_a"]["end"] = csv_row_count
        elif op == 2:
            json_dictionary["y_p"]["end"] = csv_row_count
        elif op == 3:
            json_dictionary["y_a"]["end"] = csv_row_count
        elif op == 4:
            json_dictionary["z_a"]["end"] = csv_row_count
        elif op == 5:
            json_dictionary["z_p"]["end"] = csv_row_count
        elif op == 7:
            json_dictionary["x_rot"]["end"] = csv_row_count - 20
        elif op == 9:
            json_dictionary["y_rot"]["end"] = csv_row_count - 20
        elif op == 10:
            json_dictionary["z_rot"]["end"] = csv_row_count - 20
            jsonFile.write(json.dumps(json_dictionary))
            jsonFile.close()
            csvFile.close()
            process_calibration()


def process_calibration():
    data = pd.read_csv("imu_raw_data.csv", header=0, index_col=0)
    data.head()

    section_list = pd.read_json("ferraris_session.json").T
    regions = ferraris_regions_from_section_list(data, section_list)

    cal = TurntableCalibration()
    cal_info = cal.compute(
        regions, sampling_rate_hz=20.0, from_acc_unit="m/s^2", from_gyr_unit="deg/s", comment="1"
    )
    file_path = save_calibration_info(
        cal_info, sensor_id="imu1", cal_time=datetime.now(), folder=Path("calibrations")
    )
    save_imu_cal_to_ini(file_path)

    calibrated_data = cal_info.calibrate_df(data, "m/s^2", "deg/s")

    # Plot normalized accel data
    plt.figure()
    plt.plot(norm(data.filter(like="acc"), axis=1), label="before cal")
    plt.plot(norm(calibrated_data.filter(like="acc"), axis=1), label="after cal")
    plt.title('Norm accel data')
    plt.xlabel("data number")
    plt.ylabel("acc norm [m/s^2]")
    plt.legend()
    plt.savefig('norm_accel_data.png')
    plt.show()

    # Plot normalized gyr data
    plt.figure()
    plt.plot(norm(data.filter(like="gyr"), axis=1), label="before cal")
    plt.plot(norm(calibrated_data.filter(like="gyr"), axis=1), label="after cal")
    plt.title('Norm gyr data')
    plt.xlabel("data number")
    plt.ylabel("gyr norm [deg/s]")
    plt.legend()
    plt.savefig('norm_gyr_data.png')
    plt.show()

    # Plot XY data
    plt.figure()
    plt.plot(data["acc_x"], data["acc_y"], 'b*', label='Raw Meas.')
    plt.plot(calibrated_data["acc_x"], calibrated_data["acc_y"], 'r*', label='Calibrated Meas.')
    plt.title('XY Accelerometer Data')
    plt.xlabel('X [m/s^2]')
    plt.ylabel('Y [m/s^2]')
    plt.legend()
    plt.grid()
    plt.axis('equal')
    plt.savefig('xy_accel_data.png')
    plt.show()

    # Plot YZ data
    plt.figure()
    plt.plot(data["acc_y"], data["acc_z"], 'b*', label='Raw Meas.')
    plt.plot(calibrated_data["acc_y"], calibrated_data["acc_z"], 'r*', label='Calibrated Meas.')
    plt.title('YZ Accelerometer Data')
    plt.xlabel('Y [m/s^2]')
    plt.ylabel('Z [m/s^2]')
    plt.legend()
    plt.grid()
    plt.axis('equal')
    plt.savefig('yz_accel_data.png')
    plt.show()

    # Plot XZ data
    plt.figure()
    plt.plot(data["acc_x"], data["acc_z"], 'b*', label='Raw Meas.')
    plt.plot(calibrated_data["acc_x"], calibrated_data["acc_z"], 'r*', label='Calibrated Meas.')
    plt.title('XZ Accelerometer Data')
    plt.xlabel('X [m/s^2]')
    plt.ylabel('Z [m/s^2]')
    plt.legend()
    plt.grid()
    plt.axis('equal')
    plt.savefig('xz_accel_data.png')
    plt.show()

    ferraris_regions_from_interactive_plot(data)
    gui.show_alert_dialog(1)
    # ferraris_regions_from_interactive_plot(calibrated_data)


def save_imu_cal_to_ini(file_path):
    name = re.findall('imu\\d*_.*?\\.', str(file_path))[0]
    imu_ini = open(f"iniFiles/{name}ini", 'w')
    ini_str = "[IMU];\n"
    imu_file = pd.read_json(file_path)  # Path("calibrations/imu1/Turntable/imu1_2023-07-19_16-01.json"))
    for i, row in enumerate(imu_file["K_a"]):
        for j, col in enumerate(imu_file["K_a"][i]):
            ini_str += f"K_a{i}{j}={col};\n"
    for i, row in enumerate(imu_file["R_a"]):
        for j, col in enumerate(imu_file["R_a"][i]):
            ini_str += f"R_a{i}{j}={col};\n"
    for i, col in enumerate(imu_file["b_a"]):
        ini_str += f"b_a{i}={col};\n"

    for i, row in enumerate(imu_file["K_g"]):
        for j, col in enumerate(imu_file["K_g"][i]):
            ini_str += f"K_g{i}{j}={col};\n"
    for i, row in enumerate(imu_file["R_g"]):
        for j, col in enumerate(imu_file["R_g"][i]):
            ini_str += f"R_g{i}{j}={col};\n"
    for i, row in enumerate(imu_file["K_ga"]):
        for j, col in enumerate(imu_file["K_ga"][i]):
            ini_str += f"K_ga{i}{j}={col};\n"
    for i, col in enumerate(imu_file["b_g"]):
        ini_str += f"b_g{i}={col};\n"
    imu_ini.write(ini_str)
    imu_ini.close()
    print(ini_str)


def create_bg_loop():
    def to_bg(loop):
        asyncio.set_event_loop(loop)
        try:
            loop.run_forever()
        except asyncio.CancelledError as e:
            print('CANCELLEDERROR {}'.format(e))
        finally:
            for task in asyncio.all_tasks():
                task.cancel()
            loop.run_until_complete(loop.shutdown_asyncgens())
            loop.stop()
            loop.close()

    new_loop = asyncio.new_event_loop()
    t = threading.Thread(target=to_bg, args=(new_loop,))
    t.start()
    return new_loop


async def start_loop():
    loop_WMD3 = create_bg_loop()
    asyncio.run_coroutine_threadsafe(write_to_WMD3(loop_WMD3), loop_WMD3)
    loop_imu_read = create_bg_loop()
    asyncio.run_coroutine_threadsafe(read_from_COM(loop_imu_read, "COM12", 921200), loop_imu_read)


async def write_to_WMD3(loop):
    await serial_asyncio.create_serial_connection(loop, WMD3Protocol, 'COM11', baudrate=9600)


async def read_from_COM(loop, comName, rate=9600):
    await serial_asyncio.create_serial_connection(loop, ImuSensorProtocol, comName, baudrate=rate)


def start_calibration():
    main_loop = asyncio.get_event_loop()
    main_task = main_loop.run_until_complete(start_loop())
# if __name__ == '__main__':
#     #save_imu_cal_to_ini(Path("calibrations/imu1/Turntable/imu1_2023-07-19_16-01.json"))
#     # imu_json = pd.read_json(Path("calibrations/imu1/Turntable/imu1_2023-07-19_16-01.json"))
#     # print(re.findall("imu\\d*_.*?\\.", "calibrations/imu1/Turntable/imu1_2023-07-19_16-01.json")[0])
#     #print(imu_json["K_ga"][3])
#     # process_calibration()
#     main_loop = asyncio.get_event_loop()
#     main_task = main_loop.run_until_complete(start_loop())
