import numpy as np
import imufusion # 姿勢推定のライブラリです。便利〜。
from pythonosc import dispatcher, osc_server, udp_client
import threading
import time
import sys

# コマンドライン引数の確認
if len(sys.argv) < 2:
    print("Usage: python script.py [port]") # コマンドラインに受け取る際のポート番号を指定してpythonを実行。Raspiの台数分Pythonを同時に走らせる。
    sys.exit(1)

# グローバル変数の初期化
gyro_data_l = np.zeros(3)
accel_data_l = np.zeros(3)
mag_data_l = np.zeros(3)
gyro_data_r = np.zeros(3)
accel_data_r = np.zeros(3)
mag_data_r = np.zeros(3)
sample_rate = 100  # 100 Hz

# OSCメッセージハンドラの定義
def handle_accel_l(unused_addr, x, y, z):
    global accel_data_l
    accel_data_l[:] = [x, y, z]

def handle_gyro_l(unused_addr, x, y, z):
    global gyro_data_l
    gyro_data_l[:] = [x, y, z]

def handle_mag_l(unused_addr, x, y, z):
    global mag_data_l
    mag_data_l[:] = [x, y, z]

def handle_accel_r(unused_addr, x, y, z):
    global accel_data_r
    accel_data_r[:] = [x, y, z]

def handle_gyro_r(unused_addr, x, y, z):
    global gyro_data_r
    gyro_data_r[:] = [x, y, z]

def handle_mag_r(unused_addr, x, y, z):
    global mag_data_r
    mag_data_r[:] = [x, y, z]

# AHRSアルゴリズムの初期化
ahrs_l = imufusion.Ahrs()
ahrs_r = imufusion.Ahrs()
ahrs_l.settings = ahrs_r.settings = imufusion.Settings(
    imufusion.CONVENTION_NWU,  # 地球軸の慣例 (NWU)
    60.0,  # ゲイン
    2000,  # ジャイロスコープの範囲 (deg/s)
    10,  # 加速度の拒否のしきい値 (degrees)
    10,  # 磁気の拒否のしきい値 (degrees)
    int(0.05 * sample_rate)  # 復帰トリガー期間 = 5秒
)

# OSCサーバーのアドレスとポート
ip = "192.168.10.112"
port = int(sys.argv[1])  # コマンドラインから受け取ったポート番号

# OSCサーバーの設定
disp = dispatcher.Dispatcher()
disp.map("/raspi/L/accel", handle_accel_l)
disp.map("/raspi/L/gyro", handle_gyro_l)
disp.map("/raspi/L/mag", handle_mag_l)
disp.map("/raspi/R/accel", handle_accel_r)
disp.map("/raspi/R/gyro", handle_gyro_r)
disp.map("/raspi/R/mag", handle_mag_r)

# OSCサーバーの開始
server = osc_server.ThreadingOSCUDPServer((ip, port), disp)
server_thread = threading.Thread(target=server.serve_forever)
server_thread.start()

# OSCクライアント（送信先）の初期化
send_ip = "127.0.0.1" # ローカルのTouchDesignerで後処理するので、ローカルのIPアドレス。
send_port = 8000 # ローカルのTouchDesignerで開いておくポート番号。ここ確認！
client = udp_client.SimpleUDPClient(send_ip, send_port)

# 姿勢推定とデータ送信の実行
try:
    while True:
        # センサーデータを更新
        ahrs_l.update(gyro_data_l, accel_data_l, mag_data_l, 1.0 / sample_rate)
        ahrs_r.update(gyro_data_r, accel_data_r, mag_data_r, 1.0 / sample_rate)
        
        # クォータニオンをオイラー角に変換
        euler_angles_l = ahrs_l.quaternion.to_euler()
        euler_angles_r = ahrs_r.quaternion.to_euler()

        # OSCメッセージでデータ送信
        client.send_message(f"/port{port}/posture/l/accel", [float(x) for x in accel_data_l])
        client.send_message(f"/port{port}/posture/l/angle", [float(x) for x in euler_angles_l])
        client.send_message(f"/port{port}/posture/r/accel", [float(x) for x in accel_data_r])
        client.send_message(f"/port{port}/posture/r/angle", [float(x) for x in euler_angles_r])

        # サンプルレートに合わせた待機
        time.sleep(1.0 / sample_rate)

except KeyboardInterrupt:
    server.shutdown()
    print("OSC server shutdown.")
