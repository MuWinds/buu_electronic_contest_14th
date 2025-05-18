import cv2
import numpy as np
from pupil_apriltags import Detector
import paho.mqtt.client as mqtt
import json
import time
import threading
from flask import Flask, Response

# MQTT配置
MQTT_SERVER = "localhost"
MQTT_PORT = 1883
MQTT_USER = ""
MQTT_PASSWORD = ""
CONTROL_TOPIC = "car/control"
CAMERA_TOPIC = "car/camera"

# 全局状态变量
latest_frame = None  # 全局帧缓存
stopped = False
last_nav_id = None
last_speed = (0, 0)
turning_start_time = None
is_turning = False
TURN_DURATION = 8  # 根据实际转向速度调整（单位：秒）

# AprilTag检测器初始化
detector = Detector(families="tag36h11")

# Web服务器配置
WEB_SERVER_PORT = 5000

app = Flask(__name__)


def generate_video():
    """生成视频流的生成器函数"""
    global latest_frame
    while True:
        if latest_frame is not None:
            # 将帧转换为JPEG格式
            ret, buffer = cv2.imencode(".jpg", latest_frame)
            frame = buffer.tobytes()
            yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        else:
            time.sleep(0.1)


@app.route("/video_feed")
def video_feed():
    """视频流路由"""
    return Response(
        generate_video(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


def detect_apriltag(image):
    """检测图像中的AprilTag并返回标签ID列表"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    results = detector.detect(gray)
    return results


def send_command(client: mqtt.Client, left, right):
    """发送控制命令"""
    payload = json.dumps({"left": left, "right": right})
    client.publish(CONTROL_TOPIC, payload)


def get_speed_from_nav_id(nav_id):
    """根据导航ID返回对应的速度"""
    if nav_id == 1:  # 直行
        return (300, 300)
    elif nav_id == 2:  # 左转
        return (150, 300)
    elif nav_id == 3:  # 右转
        return (300, 150)
    return (0, 0)


def draw_tags(image, results):
    """在图像上绘制AprilTag检测结果"""
    for r in results:
        # 绘制边界框
        corners = r.corners.astype(int)
        cv2.polylines(image, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

        # 绘制中心点和标签ID
        center = tuple(r.center.astype(int))
        cv2.circle(image, center, 5, (0, 0, 255), -1)
        cv2.putText(
            image,
            str(r.tag_id),
            (center[0] + 10, center[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )
    return image


def on_connect(client: mqtt.Client, userdata, flags, rc):
    """MQTT连接回调"""
    if rc == 0:
        print("成功连接MQTT服务器")
        client.subscribe(CAMERA_TOPIC)
    else:
        print(f"连接失败，错误码：{rc}")


def on_camera_message(client, userdata, msg):
    global latest_frame, stopped, last_nav_id, last_speed, turning_start_time, is_turning

    # 解码图像并处理
    nparr = np.frombuffer(msg.payload, np.uint8)
    image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

    # 检测和绘制AprilTag
    tags = detect_apriltag(image)
    processed_image = draw_tags(image.copy(), tags)
    latest_frame = processed_image

    if manual_mode:
        return

    # 处理停车逻辑
    has_stop_tag = 0 in [r.tag_id for r in tags]
    if has_stop_tag:
        if not stopped:
            stopped = True
            is_turning = False
            send_command(client, 0, 0)
            print("检测到停车标签，紧急停车！")
        return
    else:
        if stopped:
            stopped = False
            print("停车标签消失")

    # 处理导航指令
    if not stopped:
        nav_tags = [r.tag_id for r in tags if r.tag_id in {1, 2, 3}]
        current_nav_id = nav_tags[0] if nav_tags else None

        # 转向完成时恢复直行
        if is_turning and (time.time() - turning_start_time) >= TURN_DURATION:
            is_turning = False
            last_nav_id = 1  # 强制设置为直行指令
            last_speed = get_speed_from_nav_id(1)  # (300, 300)
            send_command(client, 0, 0)
            send_command(client, *last_speed)
            print(f"转向完成，恢复直行")
            return

        if current_nav_id:
            # 更新导航指令
            if current_nav_id != last_nav_id:
                is_turning = False  # 新指令打断正在进行的转向

            last_nav_id = current_nav_id
            new_speed = get_speed_from_nav_id(current_nav_id)

            if current_nav_id in [2, 3] and not is_turning:
                turning_start_time = time.time()
                is_turning = True
                print(f"开始{['左转','右转'][current_nav_id-2]}，持续{TURN_DURATION}秒")
            # 先让他停止才能原地转
            send_command(client, 0, 0)
            send_command(client, *new_speed)
            last_speed = new_speed

        elif not nav_tags and last_speed != (0, 0):
            # 没有检测到指令时保持最后速度
            send_command(client, *last_speed)


def input_listener(client):
    """手动控制输入监听"""
    global manual_mode, stopped
    manual_mode = False  # 初始化模式状态

    print("\n可用命令:")
    print("forward   - 直行")
    print("backward  - 后退")
    print("left      - 左转")
    print("right     - 右转")
    print("stop      - 急停")
    print("auto      - 切换自动模式")

    while True:
        try:
            # 使用标准input函数获取命令
            cmd = input().strip().lower()

            if cmd == "forward":
                send_command(client, 300, 300)
                manual_mode = True
                stopped = False
                print("手动模式：前进")
            elif cmd == "backward":
                send_command(client, -300, -300)
                manual_mode = True
                stopped = False
                print("手动模式：后退")
            elif cmd == "left":
                send_command(client, -150, 300)
                manual_mode = True
                stopped = False
                print("手动模式：左转")
            elif cmd == "right":
                send_command(client, 300, -150)
                manual_mode = True
                stopped = False
                print("手动模式：右转")
            elif cmd == "stop":
                send_command(client, 0, 0)
                manual_mode = True
                stopped = True
                print("手动模式：急停")
            elif cmd == "auto":
                manual_mode = False
                stopped = False
                print("切换回自动模式")
            else:
                print("无效命令，可用命令:")
                print("forward/backward/left/right/stop/auto")

        except Exception as e:
            print(f"输入错误: {str(e)}")
            break


def main():
    global latest_frame

    client = mqtt.Client()
    client.on_connect = on_connect
    client.message_callback_add(CAMERA_TOPIC, on_camera_message)

    # 启动输入监听线程
    threading.Thread(target=input_listener, args=(client,), daemon=True).start()

    try:
        client.connect(MQTT_SERVER, MQTT_PORT, 60)
        client.loop_start()
        print("系统已启动...")
        # 启动Web服务器
        print(f"启动Web服务器,访问 http://localhost:{WEB_SERVER_PORT}/video_feed")
        app.run(host="0.0.0.0", port=WEB_SERVER_PORT, threaded=True)
        # 主显示循环
        while True:
            if latest_frame is not None:
                cv2.imshow("Car Camera", latest_frame)

            # 处理按键事件
            key = cv2.waitKey(1)
            if key == 27:  # ESC退出
                break

    except KeyboardInterrupt:
        send_command(client, 0, 0)
        client.disconnect()
    finally:
        cv2.destroyAllWindows()
        print("系统已关闭")


if __name__ == "__main__":
    main()
