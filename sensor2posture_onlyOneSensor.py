import numpy as np
import imufusion # 姿勢推定に使うライブラリ
from pythonosc import dispatcher, osc_server, udp_client
import threading
import time
import sys

# コマンドライン引数の確認
if len(sys.argv) < 2:
    print("Usage: python script.py [port]")
    sys.exit(1)

# グローバル変数の初期化
gyro_data = np.zeros(3)
accel_data = np.zeros(3)
mag_data = np.zeros(3)
sample_rate = 100  # 100 Hz

# OSCメッセージハンドラの定義
def handle_accel(unused_addr, x, y, z):
    global accel_data
    accel_data[:] = [x, y, z]

def handle_gyro(unused_addr, x, y, z):
    global gyro_data
    gyro_data[:] = [x, y, z]

def handle_mag(unused_addr, x, y, z):
    global mag_data
    mag_data[:] = [x, y, z]

# AHRSアルゴリズムの初期化
ahrs = imufusion.Ahrs()
ahrs.settings = imufusion.Settings(
    imufusion.CONVENTION_NWU,  # 地球軸の慣例 (NWU)
    60.0,  # ゲイン
    2000,  # ジャイロスコープの範囲 (deg/s)
    10,  # 加速度の拒否のしきい値 (degrees)
    10,  # 磁気の拒否のしきい値 (degrees)
    int(0.05 * sample_rate)  # 復帰トリガー期間 = 5秒
)

# OSCサーバーのアドレスとポート
ip = "192.168.10.110"
port = int(sys.argv[1])  # コマンドラインから受け取ったポート番号

# OSCサーバーの設定
disp = dispatcher.Dispatcher()
disp.map("/raspi/l/accel", handle_accel)
disp.map("/raspi/l/gyro", handle_gyro)
disp.map("/raspi/l/mag", handle_mag)

# OSCサーバーの開始
server = osc_server.ThreadingOSCUDPServer((ip, port), disp)
server_thread = threading.Thread(target=server.serve_forever)
server_thread.start()

# OSCクライアント（送信先）の初期化
send_ip = "127.0.0.1" # センサー情報にフィルタをかけるのはセンサー班のMac（同じPC内）のTouchDesignerでやる。
send_port = 8000
client = udp_client.SimpleUDPClient(send_ip, send_port)

# 姿勢推定とデータ送信の実行
try:
    while True:
        # センサーデータを更新
        ahrs.update(gyro_data, accel_data, mag_data, 1.0 / sample_rate)
        
        # クォータニオンをオイラー角に変換
        euler_angles = ahrs.quaternion.to_euler()

        # クォータニオンのデータを配列に格納
        quaternion_data = [ahrs.quaternion.w, ahrs.quaternion.x, ahrs.quaternion.y, ahrs.quaternion.z]

        # 結果の出力
        print("Euler Angles:", euler_angles)
        print("Accel:", accel_data)

        # OSCメッセージでデータ送信
        client.send_message(f"/port{port}/posture/l/accel", [float(x) for x in accel_data])
        client.send_message(f"/port{port}/posture/l/angle", [float(x) for x in euler_angles])

        # サンプルレートに合わせた待機
        time.sleep(1.0 / sample_rate)

except KeyboardInterrupt:
    server.shutdown()
    print("OSC server shutdown.")
