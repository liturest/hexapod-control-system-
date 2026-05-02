#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

// 选择摄像头型号
#define CAMERA_MODEL_AI_THINKER

// 摄像头引脚配置
#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#endif

// WiFi配置
const char* ssid = "ESP32-CAM";
const char* password = "12345678";

// LED配置
#define LED_PIN 4
bool isLedOn = false;

// 服务器配置
#define SERVER_PORT 80

// 全局变量
bool isStreaming = true;
bool isImageFlipped = false;
bool isClientConnected = false;
unsigned long lastClientActivity = 0;
bool shouldRestartStream = false;

// 创建HTTP服务器
httpd_handle_t stream_httpd = NULL;

// 流媒体相关定义
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// 主页HTML
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
  <head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32-CAM 控制面板</title>
    <style>
      body { 
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
        text-align: center; 
        margin: 0; 
        padding: 20px; 
        background-color: #f5f5f5; 
      }
      .container { 
        max-width: 800px; 
        margin: 0 auto; 
        background: white; 
        padding: 20px; 
        border-radius: 10px; 
        box-shadow: 0 0 10px rgba(0,0,0,0.1); 
      }
      h1 { color: #444; }
      .button {
        background-color: #4CAF50;
        border: none;
        color: white;
        padding: 12px 24px;
        text-align: center;
        text-decoration: none;
        display: inline-block;
        font-size: 16px;
        margin: 8px 4px;
        cursor: pointer;
        border-radius: 5px;
        -webkit-tap-highlight-color: transparent;
      }
      .button-stop { background-color: #f44336; }
      .button-flip { background-color: #2196F3; }
      .button-led { background-color: #FF9800; }
      #stream-container { margin: 20px 0; }
      img { 
        max-width: 100%; 
        height: auto; 
        border: 1px solid #ddd; 
        border-radius: 4px;
        display: block;
        margin: 0 auto;
      }
      .status { 
        margin: 15px 0; 
        font-weight: bold; 
        font-size: 18px;
      }
      @media (max-width: 600px) {
        .button {
          padding: 10px 20px;
          font-size: 14px;
        }
      }
    </style>
  </head>
  <body>
    <div class="container">
      <h1>ESP32-CAM 控制面板</h1>
      
      <div id="buttons">
        <button class="button" id="streamButton">断开视频</button>
        <button class="button button-flip" id="flipButton">图像翻转</button>
        <button class="button button-led" id="ledButton">打开LED</button>
      </div>
      
      <div class="status">
        LED状态: <span id="ledStatus">关闭</span> | 
        图像状态: <span id="flipStatus">正常</span>
      </div>
      
      <div id="stream-container">
        <img id="stream" src="">
      </div>
    </div>
    
    <script>
      var streamInterval;
      var streamImg = document.getElementById('stream');
      
      // 页面可见性API检测
      document.addEventListener('visibilitychange', function() {
        if (document.visibilityState === 'hidden') {
          // 页面被隐藏时发送关闭通知
          navigator.sendBeacon('/client_close');
          stopStream();
        } else if (document.visibilityState === 'visible') {
          startStream();
        }
      });

      // 页面卸载时发送通知
      window.addEventListener('beforeunload', function() {
        navigator.sendBeacon('/client_close');
        stopStream();
      });

      // 更兼容的事件绑定方式
      document.addEventListener('DOMContentLoaded', function() {
        // 初始化按钮事件
        document.getElementById('streamButton').addEventListener('click', controlVideo);
        document.getElementById('flipButton').addEventListener('click', flipImage);
        document.getElementById('ledButton').addEventListener('click', toggleLED);
        
        // 初始状态检查
        checkStatus();
        startStream();
      });

      function checkStatus() {
        fetch('/status')
          .then(response => response.json())
          .then(data => updateUI(data))
          .catch(error => console.error('Error:', error));
      }

      function updateUI(data) {
        if (data.ledStatus) {
          document.getElementById('ledStatus').textContent = data.ledStatus;
          document.getElementById('ledButton').textContent = 
            data.ledStatus === '打开' ? '关闭LED' : '打开LED';
        }
        if (data.flipStatus) {
          document.getElementById('flipStatus').textContent = data.flipStatus;
        }
        if (data.streamStatus) {
          document.getElementById('streamButton').textContent = 
            data.streamStatus === 'on' ? '断开视频' : '视频连接';
        }
      }

      function controlVideo() {
        var button = document.getElementById('streamButton');
        var action = button.textContent === '断开视频' ? 'stop' : 'start';
        
        sendAction(action).then(() => {
          if (action === 'start') {
            startStream();
          } else {
            stopStream();
          }
          checkStatus();
        });
      }
      
      function flipImage() {
        sendAction('flip').then(() => {
          stopStream();
          setTimeout(startStream, 100); // 短暂延迟后重新开始流
          checkStatus();
        });
      }
      
      function toggleLED() {
        sendAction('led').then(() => checkStatus());
      }
      
      function sendAction(action) {
        return fetch('/action?go=' + action)
          .catch(error => console.error('Error:', error));
      }
      
      function startStream() {
        stopStream(); // 确保先停止之前的流
        streamImg.style.display = 'block';
        streamImg.src = "/stream?" + Date.now();
        streamInterval = setInterval(function() {
          streamImg.src = "/stream?" + Date.now();
        }, 100);
      }
      
      function stopStream() {
        if (streamInterval) {
          clearInterval(streamInterval);
          streamInterval = null;
        }
        streamImg.style.display = 'none';
        streamImg.src = "";
      }
    </script>
  </body>
</html>
)rawliteral";

// 处理主页请求
static esp_err_t index_handler(httpd_req_t *req) {
  isClientConnected = true;
  lastClientActivity = millis();
  Serial.println("客户端连接");
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

// 处理客户端关闭通知
static esp_err_t client_close_handler(httpd_req_t *req) {
  Serial.println("客户端断开连接");
  isClientConnected = false;
  return httpd_resp_send(req, NULL, 0);
}

// 处理状态请求
static esp_err_t status_handler(httpd_req_t *req) {
  lastClientActivity = millis();
  char status_json[150];
  snprintf(status_json, sizeof(status_json), 
           "{\"ledStatus\":\"%s\",\"flipStatus\":\"%s\",\"streamStatus\":\"%s\"}", 
           isLedOn ? "打开" : "关闭", 
           isImageFlipped ? "翻转" : "正常",
           isStreaming ? "on" : "off");
  
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_send(req, status_json, strlen(status_json));
}

// 处理动作请求
static esp_err_t action_handler(httpd_req_t *req) {
  // 验证请求是否来自活动客户端
  if (!isClientConnected) {
    Serial.println("拒绝非活动客户端的请求");
    const char* resp_str = "Client not active";
    httpd_resp_send(req, resp_str, strlen(resp_str));
    return ESP_FAIL;
  }

  lastClientActivity = millis();
  
  char* buf;
  size_t buf_len;
  char variable[32] = {0,};
  
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char*)malloc(buf_len);
    if (!buf) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "go", variable, sizeof(variable)) == ESP_OK) {
        Serial.printf("收到动作请求: %s\n", variable);
        
        if (strcmp(variable, "start") == 0) {
          isStreaming = true;
          Serial.println("视频流已开启");
        } else if (strcmp(variable, "stop") == 0) {
          isStreaming = false;
          Serial.println("视频流已停止");
        } else if (strcmp(variable, "flip") == 0) {
          isImageFlipped = !isImageFlipped;
          sensor_t *s = esp_camera_sensor_get();
          s->set_hmirror(s, isImageFlipped);
          s->set_vflip(s, isImageFlipped);
          Serial.printf("图像已%s翻转\n", isImageFlipped ? "" : "取消");
          shouldRestartStream = true;
        } else if (strcmp(variable, "led") == 0) {
          isLedOn = !isLedOn;
          digitalWrite(LED_PIN, isLedOn ? HIGH : LOW);
          Serial.printf("LED已%s\n", isLedOn ? "打开" : "关闭");
        }
      }
    }
    free(buf);
  }
  
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

// 处理视频流请求
static esp_err_t stream_handler(httpd_req_t *req) {
  lastClientActivity = millis();
  
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char part_buf[64];
  
  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    Serial.println("设置流媒体类型失败");
    return res;
  }
  
  while (true) {
    if (!isStreaming) {
      Serial.println("视频流已暂停");
      break;
    }
    
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("摄像头捕获失败");
      res = ESP_FAIL;
    } else {
      if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted) {
          Serial.println("JPEG转换失败");
          res = ESP_FAIL;
        }
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }
    
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    
    if (res != ESP_OK) {
      Serial.println("发送流数据失败");
      break;
    }
    
    // 检查是否需要重启流（如图像翻转后）
    if (shouldRestartStream) {
      shouldRestartStream = false;
      break;
    }
    
    // 检查客户端是否仍然活跃
    if (millis() - lastClientActivity > 5000) { // 5秒无活动
      Serial.println("客户端活动超时");
      break;
    }
  }
  
  return res;
}

// 初始化WiFi
void setupWiFi() {
  Serial.println("正在设置WiFi AP...");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP地址: ");
  Serial.println(IP);
}

// 初始化摄像头
void setupCamera() {
  Serial.println("正在初始化摄像头...");
  
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("摄像头初始化失败，错误代码: 0x%x\n", err);
    return;
  }
  
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA);
  Serial.println("摄像头初始化完成");
}

// 初始化LED
void setupLED() {
  Serial.println("正在初始化LED...");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.println("LED初始化完成");
}

// 启动Web服务器
void startCameraServer() {
  Serial.println("正在启动Web服务器...");
  
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = SERVER_PORT;
  config.ctrl_port = SERVER_PORT;
  config.max_open_sockets = 3;
  
  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
  };
  
  httpd_uri_t client_close_uri = {
    .uri = "/client_close",
    .method = HTTP_GET,
    .handler = client_close_handler,
    .user_ctx = NULL
  };
  
  httpd_uri_t status_uri = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_handler,
    .user_ctx = NULL
  };
  
  httpd_uri_t action_uri = {
    .uri = "/action",
    .method = HTTP_GET,
    .handler = action_handler,
    .user_ctx = NULL
  };
  
  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };
  
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
    httpd_register_uri_handler(stream_httpd, &client_close_uri);
    httpd_register_uri_handler(stream_httpd, &status_uri);
    httpd_register_uri_handler(stream_httpd, &action_uri);
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.println("Web服务器启动成功");
  } else {
    Serial.println("Web服务器启动失败");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("\n\nESP32-CAM 初始化开始...");
  
  setupLED();
  setupCamera();
  setupWiFi();
  startCameraServer();
  
  Serial.println("系统准备就绪");
}

void loop() {
  static unsigned long lastStatusTime = 0;
  
  // 每5秒输出一次状态信息
  if (millis() - lastStatusTime > 5000) {
    lastStatusTime = millis();
    Serial.printf("系统状态 - 客户端: %s, 视频流: %s, LED: %s, 图像翻转: %s\n",
                 isClientConnected ? "连接" : "断开",
                 isStreaming ? "开启" : "关闭",
                 isLedOn ? "开" : "关",
                 isImageFlipped ? "是" : "否");
  }
  
  // 如果客户端断开连接超过10秒，重置状态
  if (isClientConnected && millis() - lastClientActivity > 10000) {
    Serial.println("客户端长时间无活动，视为断开连接");
    isClientConnected = false;
    
    // 恢复到默认状态
    if (isLedOn) {
      isLedOn = false;
      digitalWrite(LED_PIN, LOW);
      Serial.println("自动关闭LED");
    }
    
    if (isImageFlipped) {
      isImageFlipped = false;
      sensor_t *s = esp_camera_sensor_get();
      s->set_hmirror(s, false);
      s->set_vflip(s, false);
      Serial.println("自动恢复图像方向");
    }
    
    if (!isStreaming) {
      isStreaming = true;
      Serial.println("自动恢复视频流");
    }
  }
  
  delay(100);
}