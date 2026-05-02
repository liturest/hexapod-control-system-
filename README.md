# 🦿 Hexapod Control System | 六足机器人控制系统
有问题请联系邮箱：h3438379874@163.com

基于 Arduino Mega 2560 与 ESP32 的 18 自由度六足机器人控制系统，集成三角步态算法、AI 语音交互、WiFi 实时视频流、多传感器环境感知等功能。

## 📋 目录

- [功能特点](#功能特点)
- [硬件架构](#硬件架构)
- [引脚连接](#引脚连接)
- [软件架构](#软件架构)
- [安装与烧录](#安装与烧录)
- [使用说明](#使用说明)
- [运动算法](#运动算法)
- [二次开发](#二次开发)
- [常见问题](#常见问题)
- [开源协议](#开源协议)

## ✨ 功能特点

### 核心运动
- **三角步态算法**：六足交替运动，支持前进/后退/左右转弯
- **18路舵机控制**：Coxa/Femur/Tibia 三关节协同，带角度安全限位
- **平滑初始化**：上电自检，腿部自动归位 HOME 姿态
- **红外遥控**：支持红外遥控器切换运动状态

### AI 语音交互
- **语音识别**：INMP441 麦克风 + 百度语音识别 API
- **大模型对话**：阿里云 LLM 大模型生成回复
- **语音合成**：百度 TTS 合成语音，MAX98357A 功放播放
- **自定义音色**：支持录入个人音色

### 视频监控
- **实时视频流**：ESP32-CAM + OV2640 摄像头
- **WiFi 热点**：独立 AP 模式，浏览器访问 `192.168.4.1` 观看画面
- **响应式界面**：TailwindCSS 设计，手机/PC 均可查看
- **远程控制**：网页端可控制 LED、图像翻转等

### 环境感知
- **超声波避障**：HC-SR04，40cm 安全距离自动转向
- **温湿度检测**：DHT11 传感器
- **人体检测**：HC-SR501 人体红外传感器
- **可燃气体**：MQ-9 可燃气体传感器

### 信息显示
- **OLED 屏幕**：128×64 SSD1306，自定义中文字库
- **状态反馈**：实时显示传感器数据与系统状态

## 🔧 硬件架构
┌─────────────────────────────────────────────────┐
│ Arduino Mega 2560 (主控) │
│ · 状态机管理 · 步态生成 · 舵机控制 │
│ · 传感器采集 · OLED显示 · 红外接收 │
└──────┬──────────────────────────────┬────────────┘
│ UART/I2C │ PWM×18
▼ ▼
┌──────┴──────────┐ ┌──────┴──────────┐
│ ESP32-WROOM-32 │ │ 18× MG996R 舵机 │
│ · AI语音交互 │ │ · Coxa×6 │
│ · 百度/阿里API │ │ · Femur×6 │
└─────────────────┘ │ · Tibia×6 │
└─────────────────┘
┌─────────────────┐
│ ESP32-CAM │
│ · WiFi视频流 │
│ · OV2640摄像头 │
└─────────────────┘

text

### 硬件清单

| 模块 | 型号 | 数量 | 用途 |
|------|------|------|------|
| 主控 | Arduino Mega 2560 | 1 | 核心控制 |
| AI协处理 | ESP32-WROOM-32 | 1 | AI语音交互 |
| 摄像头 | ESP32-CAM + OV2640 | 1 | 视频监控 |
| 舵机 | MG996R | 18 | 关节驱动 |
| 麦克风 | INMP441 | 1 | 语音采集 |
| 功放 | MAX98357A | 1 | 语音播放 |
| 超声波 | HC-SR04 | 1 | 避障测距 |
| 温湿度 | DHT11 | 1 | 环境检测 |
| 人体检测 | HC-SR501 | 1 | 人体感应 |
| 可燃气体 | MQ-9 | 1 | 气体检测 |
| OLED | SSD1306 128×64 | 1 | 信息显示 |
| 降压模块 | XL4015 | 3 | 舵机供电 |

### 供电方案

- **舵机供电**：3S 锂电池组（12.6V）→ 3×XL4015（并联均流）→ 5V/15A(额定)
- **主控供电**：9V 电池 → Arduino Mega DC 接口
- **保护措施**：1000μF 电容（尖峰抑制）+ 8W 1Ω 均流电阻 + BMS 保护板

## 🔌 引脚连接

| Arduino Mega | 外设 | 说明 |
|:---:|:---:|:---|
| D2 ~ D19 | MG996R × 18 | 18路舵机PWM信号 |
| D20 (SDA) | OLED / 传感器 I2C | I2C 数据线 |
| D21 (SCL) | OLED / 传感器 I2C | I2C 时钟线 |
| TX1/RX1 | ESP32-WROOM-32 | AI协处理通信 |
| D22 ~ D40 | 传感器 IO | 超声波/人体/气体等 |


text

### 核心算法

#### 1. 状态机 (State Machine)
```cpp
enum RobotState {
  INIT,       // 初始化
  IDLE,       // 待命
  FORWARD,    // 前进
  BACKWARD,   // 后退
  TURN_LEFT,  // 左转
  TURN_RIGHT, // 右转
  AVOID,      // 避障
  AI_MODE     // AI交互模式
};
2. 三角步态 (Tripod Gait)
六条腿分为两组 (1,3,5) 和 (2,4,6)

相位差 180°，交替抬起/落下

抬腿高度使用正弦函数实现平滑运动

text
gaitPhase = (gaitPhase + 10) % 360
腿组A: phase = gaitPhase
腿组B: phase = (gaitPhase + 180) % 360
3. 舵机安全机制
cpp
// 角度限位
int minAngles[18] = {...};  // 最小角度
int maxAngles[18] = {...};  // 最大角度
angle = constrain(angle, minAngles[i], maxAngles[i]);
📦 安装与烧录
环境要求
Arduino IDE (1.8.x 或 2.x)

安装开发板支持：

Arduino Mega 2560 (默认支持)

ESP32 开发板：文件 → 首选项 → 附加开发板管理器网址

text
https://espressif.github.io/arduino-esp32/package_esp32_index.json
安装依赖库
在 Arduino IDE 库管理器中搜索安装：

text
Servo          (Arduino 内置)
IRremote       (红外遥控)
U8g2           (OLED 显示)
DHT sensor library
WiFi           (ESP32 内置)
ESPAsyncWebServer
烧录步骤
Arduino Mega 2560 主控

text
1. 打开 hexapod-control-system/main.cpp
2. 选择开发板: Tools → Board → Arduino Mega or Mega 2560
3. 选择端口 → 上传
ESP32-WROOM-32 (AI模块)

text
1. 打开 hexapod-control-system/ESP 32-CAM code.cpp
2. 选择开发板: ESP32 Dev Module
3. 修改 API Key (见下文)
4. 选择端口 → 上传
ESP32-CAM (摄像头模块)

🎮 使用说明
开机自检
装入 3 节 18650 电池和 9V 电池

上电后腿部自动内收，2 秒后展开至 HOME 位置

听到"滴"声后进入待命状态

红外遥控操作
按键	功能
▲	前进
▼	后退
◄	左转
►	右转
OK	停止
1	AI 语音模式
2	避障模式
观看实时视频
连接 WiFi 热点：ESP32_CAM (密码是：123456)

浏览器输入：http://192.168.4.1

手机/PC 均可查看实时画面

📐 运动算法详解
抬腿高度 (正弦平滑)
text
radPhase = (phase × 2π) / 360
liftHeight = stepHeight × (1 - cos(radPhase)) / 2
phase = 0°: liftHeight = 0 (腿着地)

phase = 180°: liftHeight = stepHeight (腿最高)

Coxa 关节前进步态
text
angleOffset = (stepLength / 2) × cos(radPhase)
finalAngle = homeAngle + angleOffset
phase = 0°: 腿向前最大

phase = 180°: 腿向后最大

超声波测距
text
distance = (pulseTime × 340) / (2 × 10000)
安全距离：40cm

检测到障碍物 → 停止 → 转向 → 继续检测

转弯步态
text
左转: 前左腿向前偏, 后左腿向后偏
右转: 前右腿向前偏, 后右腿向后偏
angleOffset = (turnAngle / 2) × cos(radPhase)
🛠 二次开发
可调参数 (gait_control.h)
cpp
#define STEP_HEIGHT   15    // 抬腿高度 (度)
#define STEP_LENGTH   20    // 步幅长度 (度)
#define GAIT_SPEED    10    // 步态速度 (度/帧)
#define TURN_ANGLE    15    // 转弯角度 (度)
#define SAFE_DISTANCE 40    // 超声波安全距离 (cm)
自定义运动函数
cpp
// 在 gait_control.cpp 中添加新的运动模式
void customGait() {
  // 你的自定义步态代码
}
修改固定角度 (getFixAngles 函数)
cpp
angles[LEG_1][COXA]  = homeAngles[LEG_1][COXA] + offset;
angles[LEG_1][FEMUR] = homeAngles[LEG_1][FEMUR] + offset;
angles[LEG_1][TIBIA] = homeAngles[LEG_1][TIBIA] + offset;
⚠️ 注意事项：

不要超过舵机最大角度限制

不要修改 HOME 位置基准值

调整后先空载测试，避免舵机堵转

自定义中文字库
准备需要显示的汉字

使用字模提取工具生成点阵数据

添加到 font_chinese.h

使用 u8g2.drawUTF8() 显示

❓ 常见问题
Q: 上电后舵机不动？
A: 检查 3S 电池电压是否 >11V，XL4015 输出是否 5V，均流电阻是否连接。

Q: 舵机抖动/无力？
A: 检查电源是否充足，1000μF 电容是否焊接正确，建议在舵机电源端并联小电容。

Q: 舵机角度不对？
A: 在 servo_driver.h 中逐个调整 homeAngles 数组值。

Q: ESP32 视频访问不了 192.168.4.1？
A: 确认手机已连接 Hexapod_CAM 热点，尝试清除浏览器缓存。

Q: AI 语音无响应？
A: 检查 API Key 是否正确，网络是否有防火墙限制，串口监视器查看调试信息。

Q: 超声波避障不灵敏？
A: 调整 SAFE_DISTANCE 参数，检查超声波安装角度是否水平。

📄 开源协议
本项目基于 MIT License 开源，你可以自由使用、修改和分发代码，但需保留原始版权声明。
