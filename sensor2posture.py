import numpy as np
from ahrs.filters import Madgwick
from pythonosc import dispatcher, osc_server, udp_client
import threading
import time
import sys

# コマンドライン引数の確認
if len(sys.argv) < 2:
    print("Usage: python script.py [port]")
    sys.exit(1)

# サンプルレートとタイムステップを定義
sample_rate = 100  # Hz

# 初期センサーデータの設定
initial_gyro = np.zeros(3)
initial_accel = np.array([0, 0, 9.81])  # m/s^2
initial_mag = np.array([0.5, 0, 0])  # 仮の地磁気の値

# Madgwickフィルタのオブジェクトを初期化
madgwick_l = Madgwick(acc=initial_accel, gyr=initial_gyro, mag=initial_mag, frequency=sample_rate, gain=0.1)
madgwick_r = Madgwick(acc=initial_accel, gyr=initial_gyro, mag=initial_mag, frequency=sample_rate, gain=0.1)

# クォータニオンからオイラー角へ変換する関数
def quaternion_to_euler(quat):
    x, y, z, w = quat
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch_y = np.arcsin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)
    return np.array([roll_x, pitch_y, yaw_z])

# OSCメッセージハンドラ
def handle_sensor_data(address, args, *sensor_data):
    madgwick, data_type = args
    if 'gyro' in address:
        gyro_data = np.array(sensor_data)
        madgwick.updateIMU(gyro=gyro_data)
    elif 'accel' in address:
        accel_data = np.array(sensor_data)
        madgwick.updateIMU(acc=accel_data)
    elif 'mag' in address:
        mag_data = np.array(sensor_data)
        madgwick.updateIMU(mag=mag_data)

    # クォータニオンからオイラー角への変換と送信
    euler_angles = quaternion_to_euler(madgwick.Q)
    client.send_message(f"{address}/euler", euler_angles.tolist())

# OSCサーバーとクライアントの設定
ip = "192.168.10.112"
port = int(sys.argv[1])  # コマンドラインから受け取ったポート番号
client_ip = "127.0.0.1"  # 送信先IP
client_port = 8000  # 送信先ポート

disp = dispatcher.Dispatcher()
disp.map("/raspi/L/*", handle_sensor_data, (madgwick_l, 'L'))
disp.map("/raspi/R/*", handle_sensor_data, (madgwick_r, 'R'))
server = osc_server.ThreadingOSCUDPServer((ip, port), disp)
client = udp_client.SimpleUDPClient(client_ip, client_port)

# サーバースレッドを開始
server_thread = threading.Thread(target=server.serve_forever)
server_thread.start()

# プログラムが終了するのを待機
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    server.shutdown()
    print("OSC server shutdown.")
