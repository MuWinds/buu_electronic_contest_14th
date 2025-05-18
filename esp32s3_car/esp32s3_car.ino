#include <WiFi.h>
#include <PubSubClient.h>
#include "app_motor_uart.h"
#include <ArduinoJson.h>
#include "esp_camera.h"

#define UPLOAD_DATA 3
#define MOTOR_TYPE 2
// 根据接线定义摄像头引脚
#define CAMERA_MODEL_ESP32S3_CUSTOM
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 15
#define SIOD_GPIO_NUM 4
#define SIOC_GPIO_NUM 5
 
#define Y2_GPIO_NUM 11
#define Y3_GPIO_NUM 9
#define Y4_GPIO_NUM 8
#define Y5_GPIO_NUM 10
#define Y6_GPIO_NUM 12
#define Y7_GPIO_NUM 18
#define Y8_GPIO_NUM 17
#define Y9_GPIO_NUM 16
 
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 7
#define PCLK_GPIO_NUM 13

// 全局变量用于控制发送频率
const int SPEED_UPDATE_INTERVAL = 100; // 每100ms更新一次速度
int lastLeftSpeed = 0;
int lastRightSpeed = 0;
unsigned long lastFrameTime = 0;
unsigned long lastSpeedUpdate = 0;
// 电机速度变量
int left_speed = 0;
int right_speed = 0;
const char *ssid = "HONOR_Magic6";
const char *wifi_password = "37gumesvjwyvwfc";
const char *mqtt_server = "192.168.237.252";
const char *mqtt_username = "";
const char *mqtt_password = "";
WiFiClient espClient;
PubSubClient client(espClient);
const int mqtt_port = 1883;

void set_wifi()
{
  delay(10);
  Serial.print("connecting...");
  Serial.println(ssid);
  WiFi.begin(ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("wifi connect success!");
  Serial.println("address:");
  Serial.println(WiFi.localIP());
}

void reconnect()
{
  while (!client.connected())
  {
    Serial.print("try to connect mqtt!");
    if (client.connect("ESPClient", mqtt_username, mqtt_password))
    {
      Serial.println("success!");
      client.subscribe("car/control"); // 订阅控制主题
    }
    else
    {
      Serial.print("fail, rc=");
      Serial.print(client.state());
      Serial.println(" retry in 5s");
      delay(5000);
    }
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("received message:[");
  Serial.print(topic);
  Serial.print("]: ");

  // 解析JSON
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload, length);

  if (error)
  {
    Serial.print("json fail!");
    Serial.println(error.c_str());
    return;
  }

  // 提取速度值
  left_speed = doc["left"] | 0; // 默认0
  right_speed = doc["right"] | 0;

  Serial.print("left speed: ");
  Serial.print(left_speed);
  Serial.print(", right speed:");
  Serial.println(right_speed);
}

void setup()
{
  Serial.begin(115200);
  set_wifi();
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QQVGA;
  config.pixel_format = PIXFORMAT_JPEG; 
  config.grab_mode = config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.jpeg_quality = 30;
  config.fb_count = 1;
  // 初始化摄像头
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("camera init fail!0x%x", err);
    return;
  }
  uart1_init();
  client.setBufferSize(1024 * 500); // 根据图像大小调整缓冲区
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  send_upload_data(false, false, false);
  delay(10);

  send_motor_type(MOTOR_310); // 使用枚举值
  delay(100);
  send_pulse_phase(20);
  delay(100);
  send_pulse_line(13);
  delay(100);
  send_wheel_diameter(48.00);
  delay(100);
  send_motor_deadzone(1300);
  delay(100);

  send_upload_data(false, false, true);
  delay(10);
}

void loop() {
  unsigned long currentTime = millis();

  // 维持MQTT连接 
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // 处理电机反馈数据
  while (motorSerial.available()) {
    Deal_Control_Rxtemp(motorSerial.read());
  }

  // 带条件的速度控制（定期发送，或者速度变化就发送）
  if ((currentTime - lastSpeedUpdate >= SPEED_UPDATE_INTERVAL) || 
      (left_speed != lastLeftSpeed) || 
      (right_speed != lastRightSpeed)) {
    Contrl_Speed(left_speed, left_speed, right_speed, right_speed);
    lastLeftSpeed = left_speed;
    lastRightSpeed = right_speed;
    lastSpeedUpdate = currentTime;
  }

  Deal_data_real(); // 保持必要的数据处理

  // 图像传输优化（降低到10FPS）
  if (currentTime - lastFrameTime >= 100) { // 100ms间隔
    lastFrameTime = currentTime;
    
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Failed to get frame");
    } else {
      // 非必要不打印调试信息
      if (!client.publish("car/camera", fb->buf, fb->len)) {
        Serial.println("Image Send Failed");
      }
      esp_camera_fb_return(fb);
    }
  }
}