#include <Servo.h>
#include <IRremote.h>
#include <DHT.h>
#include <avr/wdt.h>

// 定义速度级别
enum SpeedLevel {
  SLOW = 15,          // 慢速
  MEDIUM_SPEED = 55,  // 中速
  FAST = 80           // 高速
};

// 定义常量
#define FRAME_TIME_MS 40                      // 帧时间间隔(ms)
#define DEBOUNCE_TIME 100                     // 红外遥控器消抖时间(ms)
#define COMMAND_DURATION 6000                 // 命令持续时间
#define INITIAL_DELAY 1000                    // 初始化延迟时间(ms)
#define SERVO_NUM 18                          // 舵机数量
#define MOVEMENT_CYCLES 4                     // 运动周期数
#define PHASE_INCREMENT 20                    // 相位增量，原为8，现减小为4
#define TURN_PHASE_INCREMENT 18               // 转向时相位增量（更快）
#define OBSTACLE_DISTANCE 20                  // 障碍物检测距离(cm)改为20cm
#define DHTTYPE DHT11                         // DHT11传感器类型
#define STANCE_SETTLE_TIME 60                 // 支撑腿稳定时间
#define COXA_SWING_FACTOR 2.0                 // 摆动腿Coxa运动因子
#define COXA_STANCE_FACTOR 1.6                // 支撑腿Coxa运动因子
#define TURNING_COXA_FACTOR 2.5               // 转弯时Coxa角度因子
#define SMOOTHING_FACTOR 0.6                  // 平滑系数
#define STABILITY_THRESHOLD 20                // 稳定性阈值
#define TURNING_STEP_LENGTH 10                // 转弯步长
#define TURNING_STEP_HEIGHT 32                // 转弯时抬腿高度
#define WATCHDOG_TIMEOUT 8000                 // 看门狗超时时间(ms)
#define WATCHDOG_DISABLED true                // 看门狗禁用标志
#define MIN_COXA_ANGLE 0                      // 最小舵机角度
#define MAX_COXA_ANGLE 180                    // 最大舵机角度
#define COXA_LIFT_FACTOR 0.5                  // Coxa抬升系数
#define TURN_90_DEGREE_CYCLES 15              // 90度转弯需要的周期数
#define MOVEMENT_CYCLES_AFTER_DETECTION 4     // 超声波探测后连续运动的周期数
#define ULTRASONIC_CHECK_INTERVAL 200         // 超声波检测间隔(ms)
#define TURN_DURATION 3000                    // 转向持续时间(ms)
#define TURN_ANGLE 30                         // 转向角度(度)
#define ULTRASONIC_STEP_HEIGHT 30             // 超声波模式下抬腿高度
#define ULTRASONIC_STEP_LENGTH 18             // 超声波模式下步长
#define ULTRASONIC_TURN_ANGLE 15              // 超声波转向角度
#define ULTRASONIC_LIFT_HEIGHT 15             // 抬腿高度减小
#define ULTRASONIC_SPEED MEDIUM_SPEED         // 超声波模式速度
#define IR_DEBOUNCE_TIME 200                  // 红外消抖时间(ms)
#define MIN_COMMAND_INTERVAL 500              // 最小命令间隔时间(ms)
#define MAX_REPEAT_COUNT 5                    // 最大连续重复命令次数
#define ULTRASONIC_MOVE_DURATION 3000         // 前进持续时间(ms)
#define ULTRASONIC_TURN_DURATION 3000         // 左转持续时间(ms)
#define MANUAL_TURNING_COXA_FACTOR 2.5f       // 手动模式转向因子
#define ULTRASONIC_TURNING_FACTOR 1.2f        // 超声波模式转向因子（更平缓）
#define ULTRASONIC_TURN_ANGLE_OUTER 15 // 外侧腿最大转向角 
#define ULTRASONIC_TURN_ANGLE_INNER 10 // 内侧腿最大转向角
unsigned long ultrasonicActionStartTime = 0;  // 记录当前动作开始时间
bool isTurningLeft = false;                   // 是否正在左转
const int BUZZER_PIN = 19;                    // 蜂鸣器引脚
const unsigned long BUZZER_DURATION = 1000;   // 蜂鸣器响铃持续时间(ms)
#ifdef DEBUG_GAIT
#define DEBUG_GAIT  // 注释此行可关闭详细调试输出
#endif
#ifdef IR_PIN_PULLUP
pinMode(RECV_PIN, INPUT_PULLUP);
#endif

unsigned int frameTimeMs = 60;  // 默认40ms（可动态调整）
float TFORWARD_COXA_LIFT_FACTORS[6] = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 };
float TLEFT_TURN_COXA_LIFT_FACTORS[6] = { 0.20, 0.30, 0.20, 0.30, 0.20, 0.30 };

// 舵机引脚定义
const int COXA1_SERVO = 2;
const int FEMUR1_SERVO = 3;
const int TIBIA1_SERVO = 4;
const int COXA2_SERVO = 5;
const int FEMUR2_SERVO = 6;
const int TIBIA2_SERVO = 7;
const int COXA3_SERVO = 8;
const int FEMUR3_SERVO = 9;
const int TIBIA3_SERVO = 10;
const int COXA4_SERVO = 11;
const int FEMUR4_SERVO = 12;
const int TIBIA4_SERVO = 13;
const int COXA5_SERVO = 46;
const int FEMUR5_SERVO = 45;
const int TIBIA5_SERVO = 44;
const int COXA6_SERVO = A0;
const int FEMUR6_SERVO = A1;
const int TIBIA6_SERVO = A2;
// 传感器引脚定义
const int TRIG_PIN = 17;    // HC-SR04触发引脚
const int ECHO_PIN = 18;    // HC-SR04回声引脚
const int DHT_PIN = 14;     // DHT11数据引脚
const int MQ9_DO_PIN = 15;  // MQ-9数字输出引脚
const int PIR_PIN = 16;     // HC-SR501人体检测引脚
const int RECV_PIN = 22;    // 红外接收引脚

// 定义舵机引脚数组（按顺序：Coxa1, Femur1, Tibia1, Coxa2, ...）
const int servoPins[SERVO_NUM] = {
  COXA1_SERVO, FEMUR1_SERVO, TIBIA1_SERVO,
  COXA2_SERVO, FEMUR2_SERVO, TIBIA2_SERVO,
  COXA3_SERVO, FEMUR3_SERVO, TIBIA3_SERVO,
  COXA4_SERVO, FEMUR4_SERVO, TIBIA4_SERVO,
  COXA5_SERVO, FEMUR5_SERVO, TIBIA5_SERVO,
  COXA6_SERVO, FEMUR6_SERVO, TIBIA6_SERVO
};

// 定义状态枚举
enum RobotState {
  INITIALIZING,
  STOP,
  FORWARD,
  BACKWARD,
  LEFT_TURN,
  RIGHT_TURN,
  RETURNING_HOME,
  ERROR,
  ULTRASONIC_INIT,       // 超声波模式初始化
  ULTRASONIC_MOVING,     // 超声波前进状态
  ULTRASONIC_TURNING,    // 超声波转向中
  ULTRASONIC_TRANSITION  // 状态转换过渡期
};

// 定义操作模式
enum OperationMode {
  MANUAL_MODE,     // 手动模式
  ULTRASONIC_MODE  // 超声波模式
};

// 前进模式的抬腿因子
const float FORWARD_COXA_LIFT_FACTORS[6] = {
  -0.1,  // 腿1
  -0.1,  // 腿2
  0.01,  // 腿3
  -0.1,  // 腿4
  -0.1,  // 腿5
  2.0    // 腿6
};

const float FORWARD_TIBIA_LIFT_FACTORS[6] = {
  1.5,   // 腿1
  -1.0,  // 腿2
  1.5,   // 腿3
  -1.5,  // 腿4
  1.0,   // 腿5
  -1.5   // 腿6
};

// 后退模式的抬腿因子
const float BACKWARD_COXA_LIFT_FACTORS[6] = {
  0.0,   // 腿1
  -0.1,  // 腿2
  0.1,   // 腿3
  -0.4,  // 腿4
  -0.1,  // 腿5
  0.5    // 腿6
};

const float BACKWARD_TIBIA_LIFT_FACTORS[6] = {
  1.5,   // 腿1
  -1.0,  // 腿2
  1.5,   // 腿3
  -1.5,  // 腿4
  1.0,   // 腿5
  -1.5   // 腿6
};

// 左转模式的抬腿因子
const float LEFT_TURN_COXA_LIFT_FACTORS[6] = {
  0.20,  // 腿1 (内侧)
  0.30,  // 腿2 (外侧)
  0.20,  // 腿3 (内侧)
  0.30,  // 腿4 (外侧)
  0.20,  // 腿5 (内侧)
  0.30   // 腿6 (外侧)
};

const float LEFT_TURN_TIBIA_LIFT_FACTORS[6] = {
  1.5,   // 腿1
  -1.0,  // 腿2
  1.5,   // 腿3
  -1.5,  // 腿4
  1.0,   // 腿5
  -1.5   // 腿6
};

// 右转模式的抬腿因子
const float RIGHT_TURN_COXA_LIFT_FACTORS[6] = {
  0.30,  // 腿1 (外侧)
  0.20,  // 腿2 (内侧)
  0.30,  // 腿3 (外侧)
  0.20,  // 腿4 (内侧)
  0.30,  // 腿5 (外侧)
  0.20   // 腿6 (内侧)
};

const float RIGHT_TURN_TIBIA_LIFT_FACTORS[6] = {
  1.5,   // 腿1
  -1.0,  // 腿2
  1.5,   // 腿3
  -1.5,  // 腿4
  1.0,   // 腿5
  -1.5   // 腿6
};

// 超声波模式专用抬腿因子
const float ULTRASONIC_FORWARD_COXA_LIFT_FACTORS[6] = {
  0.1,  // 腿1
  0.1,  // 腿2
  0.1,  // 腿3
  0.1,  // 腿4
  0.1,  // 腿5
  0.1   // 腿6
};

const float ULTRASONIC_FORWARD_TIBIA_LIFT_FACTORS[6] = {
  1.0,   // 腿1
  -1.0,  // 腿2
  1.0,   // 腿3
  -1.0,  // 腿4
  1.0,   // 腿5
  -1.0   // 腿6
};

const float ULTRASONIC_TURN_COXA_LIFT_FACTORS[6] = {
  0.25,  // 腿1 (内侧)
  0.35,  // 腿2 (外侧)
  0.25,  // 腿3 (内侧)
  0.35,  // 腿4 (外侧)
  0.25,  // 腿5 (内侧)
  0.35   // 腿6 (外侧)
};

const float ULTRASONIC_TURN_TIBIA_LIFT_FACTORS[6] = {
  1.2,   // 腿1
  -1.2,  // 腿2
  1.2,   // 腿3
  -1.2,  // 腿4
  1.2,   // 腿5
  -1.2   // 腿6
};

// 超声波左转时的舵机方向修正 (Coxa/Femur/Tibia)
/*const int ultrasonicLeftTurnServoDirection[SERVO_NUM] = {
  1, -1, 1,  // 腿1: Coxa(正), Femur(负), Tibia(正)
  1, -1, 1,  // 腿2: Coxa(负), Femur(负), Tibia(正)
  1, -1, 1,  // 腿3
  -1, -1, 1,  // 腿4
  -1, -1, 1,  // 腿5
  -1, -1, 1   // 腿6
};*/

// 三角步态分组 - 明确指定两组腿
const int TRIPOD_GROUP1[3] = { 0, 2, 4 };  // 腿1, 3, 5
const int TRIPOD_GROUP2[3] = { 1, 3, 5 };  // 腿2, 4, 6

// 创建传感器对象
Servo servos[SERVO_NUM];
IRrecv irrecv(RECV_PIN);
decode_results results;
DHT dht(DHT_PIN, DHTTYPE);

// 新增超声波模式专用变量
bool ultrasonicInitialized = false;
int turnCycles = 0;
bool turningCompleted = false;
int remainingMovementCycles = 0;        // 剩余运动周期计数
bool needReturnHomeBeforeTurn = false;  // 需要先返回HOME再转向的标志
unsigned long lastUltrasonicCheck = 0;
int turnCyclesCount = 0;                   // 转向周期计数器
const int TURN_CYCLES_90_DEGREE = 15;      // 90度转向需要的周期数
#define ULTRASONIC_TURN_PREPARE_TIME 300   // 转向准备时间(ms)
#define ULTRASONIC_TURN_COMPLETE_TIME 500  // 转向完成时间(ms)
#define SAFE_PHASE_TOLERANCE 15            // 安全相位点容差范围(度)


// 全局变量
RobotState currentAction = INITIALIZING;
RobotState lastAction = INITIALIZING;      // 记录上一个动作状态
RobotState previousAction = INITIALIZING;  // 新增：记录前一个动作状态
OperationMode currentMode = MANUAL_MODE;
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long lastCommandTime = 0;
unsigned long commandStartTime = 0;
unsigned long returnHomeStartTime = 0;
unsigned long lastSerialOutputTime = 0;
unsigned long lastSensorReadTime = 0;
unsigned long obstacleDetectedTime = 0;
int stepHeight = 28;                            // 抬腿高度
int stepLength = 25;                            // 步长
int turnAngle = 15;                             // 转弯角度
SpeedLevel speed = MEDIUM_SPEED;                // 默认使用中速
int gaitPhase = 0;                              // 步态相位
int movementCycleCount = 0;                     // 运动周期数
bool systemInitialized = false;                 // 系统初始化完成标志
bool obstacleDetected = false;                  // 是否检测到障碍物
int initialHeightOffset = 0;                    // 初始高度偏移量
int obstacleTurnCycles = 0;                     // 障碍物转向周期计数
bool stanceLegSettled = false;                  // 支撑腿是否稳定
unsigned long stanceStartTime = 0;              // 支撑阶段开始时间
int currentCoxaAngles[SERVO_NUM / 3];           // 记录当前Coxa角度，用于平滑过渡
int currentFemurAngles[SERVO_NUM / 3];          // 记录当前Femur角度
bool lastCommandWasStop = false;                // 记录上一次命令是否为停止
bool transitioningState = false;                // 状态转换标志
int stabilityCounter = 0;                       // 状态稳定计数器
volatile unsigned long watchdogResetCount = 0;  // 看门狗复位计数
unsigned long resetProtection = 0;              // 系统保护计数器
bool irErrorDetected = false;                   // 红外错误标志
int irErrorCount = 0;                           // 红外错误计数
unsigned long lastValidCommandTime = 0;         // 记录最后一次有效命令的时间戳
unsigned long lastValidCommand = 0;             // 记录最后一次有效命令的值
float lastValidDistance = 50.0;                 // 超声波距离滤波
bool firstCycleAfterStateChange = false;        // 状态变化后的第一个周期标志
bool initializedFromStop = false;               // 从停止状态初始化标志
int previousGaitPhase = 0;                      // 记录上一个步态相位
bool firstMovementCycle = true;                 // 第一个运动周期标志
unsigned long lastTripodGaitTime = 0;           // 记录最后一次执行 tripodGait 的时间
int movementDirection = 1;                      // 运动方向（1:前进，-1:后退）
int turningDirection = 1;                       // 转弯方向（1:左转，-1:右转）
int lastMovementState = -1;                     // 记录上一个运动状态，用于状态转换处理
bool resetPhaseOnStateChange = false;           // 状态变化时是否重置相位标志
bool forcePhaseReset = false;                   // 强制相位重置标志
int criticalPhaseCounter = 0;                   // 关键相位计数器
bool phaseTransitioning = false;                // 相位转换标志
bool cycleCompleted = false;                    // 周期完成标志
int consecutiveFullCycles = 0;                  // 连续完整周期计数
int gaitPhaseOffset = 0;                        // 步态相位偏移量，用于平滑状态切换
bool maintainMovementParameters = false;        // 保持运动参数标志
int lastMovementCommand = -1;                   // 记录上一次运动命令
int coxaAngleHistory[6][10];                    // 记录每个腿的Coxa角度历史，用于调试
int coxaHistoryIndex = 0;                       // 历史索引
int stateTransitionCounter = 0;                 // 状态转换计数器
bool stateTransitionCompleted = false;          // 状态转换完成标志
int consecutiveStateChecks = 0;                 // 连续状态检查计数
bool safeToTransition = true;                   // 是否安全转换状态
bool ignoreLegStabilityCheck = false;           // 忽略腿部稳定性检查标志
bool commandLocked = false;                     // 命令锁定标志
int requiredCycles = 5;                         // 执行命令所需的完整周期数
static unsigned long accumulatedPhase = 0;      // 避免相位归零
bool commandHandled = false;                    // 新增：命令处理标志
int turnStep = 0;
unsigned long lastValidIRTime = 0;
unsigned long lastIRCommand = 0;
int irRepeatCount = 0;
bool validCommand = false;  // 添加这行
bool isUltrasonicTurning = false;
unsigned long lastModeSwitchTime = 0;
bool needDelayAfterModeSwitch = false;
unsigned long lastStateChangeTime = 0;
const unsigned long STATE_TRANSITION_TIMEOUT = 1000;  // 状态转换超时1秒
unsigned long buzzerStartTime = 0;                    // 蜂鸣器开始响铃时间
bool buzzerActive = false;                            // 蜂鸣器是否正在响铃

void handleUltrasonicMode();
void processModeSwitchCommand(unsigned long command);
void processManualModeCommand(unsigned long command);
void processUltrasonicModeCommand(unsigned long command);
void resetMovementState();
void prepareForMovement();
void initializeUltrasonicMode();
void handleUltrasonicMoving(float* lastValidDistance);
void handleUltrasonicTurning();
void executeTurnStep(int step, int totalSteps);

// 定义HOME位置角度
const int homeAngles[SERVO_NUM] = {
  90, 70, 70,  // 所有腿统一初始角度
  90, 70, 70,  // 调整Coxa初始角度，优化对称性
  90, 70, 70,
  90, 70, 70,
  90, 70, 70,
  90, 70, 70  // 调整第六条腿的Femur角度，优化初始姿态
};

// 定义舵机方向修正
const int walkServoDirection[SERVO_NUM] = {
  -1, -1, 1,  // 腿1: Coxa, Femur, Tibia (前进时Coxa为-1)
  1, -1, 1,   // 腿2
  -1, -1, 1,  // 腿3
  -1, -1, 1,  // 腿4
  1, -1, 1,   // 腿5
  -1, -1, 1   // 腿6
};

// 左转时的舵机方向修正 (Coxa/Femur/Tibia)
const int leftTurnServoDirection[SERVO_NUM] = {
  1, -1, 1,  // 腿1: Coxa(正), Femur(负), Tibia(正)
  1, -1, 1,  // 腿2: Coxa(负), Femur(负), Tibia(正)
  1, -1, 1,  // 腿3
  1, -1, 1,  // 腿4
  1, -1, 1,  // 腿5
  1, -1, 1   // 腿6
};

// 右转时的舵机方向修正
const int rightTurnServoDirection[SERVO_NUM] = {
  -1, -1, 1,  // 腿1: Coxa(负), Femur(负), Tibia(正)
  -1, -1, 1,  // 腿2: Coxa(正), Femur(负), Tibia(正)
  -1, -1, 1,  // 腿3
  -1, -1, 1,  // 腿4
  -1, -1, 1,  // 腿5
  -1, -1, 1   // 腿6
};

// 超声波模式专用函数
void setUltrasonicForwardParams() {
  stepLength = ULTRASONIC_STEP_LENGTH;
  stepHeight = ULTRASONIC_STEP_HEIGHT;
  speed = ULTRASONIC_SPEED;
  movementDirection = 1;  // 前进方向
}

// 平滑初始化函数
void smoothInitialize() {
  if (systemInitialized) return;  // 只初始化一次

  // 初始化所有舵机
  for (int i = 0; i < SERVO_NUM; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(homeAngles[i]);
    // 初始化当前Coxa和Femur角度
    if (i % 3 == 0) {
      currentCoxaAngles[i / 3] = homeAngles[i];
    }
    if (i % 3 == 1) {
      currentFemurAngles[i / 3] = homeAngles[i];
    }
    delay(40);                            // 增加延迟时间，使初始化更慢
    if (!WATCHDOG_DISABLED) wdt_reset();  // 每次舵机初始化后重置看门狗
  }

  // 初始化传感器引脚
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MQ9_DO_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);

  // 初始化DHT11
  dht.begin();

  // 初始化看门狗定时器
  //if (!WATCHDOG_DISABLED) {
  //wdt_disable();  // 禁用看门狗，直到系统完全初始化
  //wdt_enable(WDTO_4S);  // 设置看门狗超时为4秒
  //}

  initialHeightOffset = 0;
  currentAction = STOP;
  transitioningState = false;          // 重置状态转换标志
  stabilityCounter = 0;                // 重置稳定计数器
  systemInitialized = true;            // 设置初始化完成标志
  initializedFromStop = true;          // 设置从停止状态初始化标志
  lastMovementState = -1;              // 重置上一个运动状态
  resetPhaseOnStateChange = false;     // 重置相位标志
  forcePhaseReset = false;             // 重置强制相位标志
  criticalPhaseCounter = 0;            // 重置关键相位计数器
  phaseTransitioning = false;          // 重置相位转换标志
  gaitPhaseOffset = 0;                 // 重置相位偏移
  maintainMovementParameters = false;  // 重置保持运动参数标志
  lastMovementCommand = -1;            // 重置上一次运动命令
  stateTransitionCounter = 0;          // 重置状态转换计数器
  stateTransitionCompleted = true;     // 设置状态转换完成
  consecutiveStateChecks = 0;          // 重置连续状态检查计数
  safeToTransition = true;             // 设置安全转换状态

  // 初始化Coxa角度历史
  for (int leg = 0; leg < 6; leg++) {
    for (int i = 0; i < 10; i++) {
      coxaAngleHistory[leg][i] = homeAngles[leg * 3];
    }
  }
}

// 站起函数
void stand() {
  for (int i = 0; i < 6; i++) {
    int femurIndex = i * 3 + 1;
    int tibiaIndex = i * 3 + 2;
    servos[femurIndex].write(homeAngles[femurIndex] - initialHeightOffset);
    servos[tibiaIndex].write(homeAngles[tibiaIndex] + initialHeightOffset);
  }
}

// 返回HOME位置
bool returnToHome() {
  static int progress = 0;
  static bool inProgress = false;

  if (!inProgress) {
    progress = 0;
    inProgress = true;
    Serial.println("返回HOME位置...");
  }

  if (progress < 100) {
    progress += 2;  // 更慢的返回速度确保稳定
    for (int i = 0; i < SERVO_NUM; i++) {
      int targetAngle = homeAngles[i];
      servos[i].write(targetAngle);
      delay(15);  // 增加延迟确保到位
    }
    return false;
  } else {
    inProgress = false;
    Serial.println("已返回HOME位置");
    return true;
  }
}

// 设置前进步态参数
void setForwardParams(SpeedLevel speedLevel) {
  // 仅在参数变化或首次设置时更新
  if (!maintainMovementParameters || stepLength != 18 || speed != speedLevel) {
    stepLength = 18;  // 前进步长
    stepHeight = 25;  // 前进步态的抬腿高度
    speed = speedLevel;
    movementDirection = 1;              // 设置前进方向
    maintainMovementParameters = true;  // 设置保持运动参数标志
  }
}

// 设置后退步态参数
void setBackwardParams(SpeedLevel speedLevel) {
  // 仅在参数变化或首次设置时更新
  if (!maintainMovementParameters || stepLength != 14 || speed != speedLevel) {
    stepLength = 18;  // 后退步长
    stepHeight = 25;  // 后退步态的抬腿高度
    speed = speedLevel;
    movementDirection = -1;             // 设置后退方向
    maintainMovementParameters = true;  // 设置保持运动参数标志
  }
}

// 设置左转步态参数
void setLeftTurnParams(SpeedLevel speedLevel) {
  turnAngle = 15;  // 左转角度
  speed = speedLevel;

  // 超声波模式使用更高的抬腿高度
  if (currentMode == ULTRASONIC_MODE) {
    stepHeight = 30;  // 超声波模式抬腿更高
  } else {
    stepHeight = 25;  // 手动模式默认高度
  }

  turningDirection = -1;  // 左转方向
  maintainMovementParameters = true;
}

// 设置右转步态参数
void setRightTurnParams(SpeedLevel speedLevel) {
  // 仅在参数变化或首次设置时更新
  if (!maintainMovementParameters || turnAngle != 10 || speed != speedLevel) {
    turnAngle = 15;   // 右转角度
    stepHeight = 25;  // 右转步态的抬腿高度
    speed = speedLevel;
    turningDirection = 1;               // 设置右转方向
    maintainMovementParameters = true;  // 设置保持运动参数标志
  }
}

int calculateCoxaAngle(int legIndex, int phase) {
  // 基础Coxa角度
  int coxaAngle = homeAngles[legIndex * 3];

  // 超声波模式专用处理
  if (currentMode == ULTRASONIC_MODE) {
    float coxaLiftFactor;
    float coxaMovementFactor = 1.0;  // 超声波模式运动幅度因子

    // 根据当前动作选择抬腿因子
    switch (currentAction) {
      case ULTRASONIC_MOVING:
        coxaLiftFactor = ULTRASONIC_FORWARD_COXA_LIFT_FACTORS[legIndex] * coxaMovementFactor;
        break;
      case ULTRASONIC_TURNING:
        coxaLiftFactor = ULTRASONIC_TURN_COXA_LIFT_FACTORS[legIndex] * coxaMovementFactor;
        break;
      default:
        coxaLiftFactor = 0.1;
        break;
    }

    // 确定腿所属的三脚架组
    bool isGroup1 = (legIndex == 0 || legIndex == 2 || legIndex == 4);

    // 计算归一化相位（0 - 360度）
    int normalizedPhase = phase % 360;

    // 计算当前步态周期中的相位比例 (0.0 - 1.0)
    float phaseRatio = normalizedPhase / 360.0f;

    // 前进运动
    if (currentAction == ULTRASONIC_MOVING) {
      // 确定腿是在摆动阶段还是支撑阶段
      bool isSwingLeg;
      if (isGroup1) {
        isSwingLeg = (normalizedPhase < 180);
      } else {
        isSwingLeg = (normalizedPhase >= 180);
      }

      // 归一化摆动相位（0 - 180度）
      int swingPhase = isSwingLeg ? normalizedPhase : (normalizedPhase - 180);

      // 使用正弦函数计算平滑的Coxa角度
      float angleOffset;
      if (isSwingLeg) {
        // 摆动阶段 - 向前移动
        angleOffset = COXA_SWING_FACTOR * ULTRASONIC_STEP_LENGTH * sin(radians(swingPhase));
      } else {
        // 支撑阶段 - 向后移动
        angleOffset = COXA_STANCE_FACTOR * ULTRASONIC_STEP_LENGTH * sin(radians(swingPhase + 180));
      }

      // 调整方向（前进）
      coxaAngle += angleOffset * 1;  // 超声波模式只前进

      // 应用该腿的Coxa抬高度数因子
      if (isSwingLeg) {
        coxaAngle += coxaLiftFactor * ULTRASONIC_STEP_HEIGHT * sin(radians(swingPhase));
      }
    }
    // 转向运动
    else if (currentAction == ULTRASONIC_TURNING) {
      // 左转方向
      float turnDirection = -1.0f;  // 左转为-1

      // 区分内外侧腿（外侧腿运动幅度更大）
      bool isOuterLeg = (legIndex % 2 == 1);  // 左转时右侧腿(2/4/6)为外侧

      // 设置幅度系数（外侧腿1.5，内侧腿0.8）
      float amplitude = isOuterLeg ? 1.5f : 0.8f;

      // 基础转向角度（30°为基准值）
      float baseTurnAngle = 30.0f * turnDirection * amplitude;

      // 添加周期性微调（使转向更自然）
      float cycleVariation = sin(2 * PI * phaseRatio) * 5.0f;

      // 应用最终角度
      coxaAngle += baseTurnAngle + cycleVariation;
    }

    // 抬腿阶段逻辑
    bool isSwingPhase = (isGroup1 && normalizedPhase < 180) || (!isGroup1 && normalizedPhase >= 180);
    if (isSwingPhase) {
      int swingPhase = isGroup1 ? normalizedPhase : (normalizedPhase - 180);
      float liftFactor = sin(radians(swingPhase));
      coxaAngle += coxaLiftFactor * ULTRASONIC_STEP_HEIGHT * liftFactor;
    }

    // 角度安全限制
    coxaAngle = constrain(coxaAngle, MIN_COXA_ANGLE, MAX_COXA_ANGLE);
    return coxaAngle;
  }

  // 以下是原有手动模式的计算逻辑（保持不变）
  float coxaMovementFactor = 1.0;
  float coxaLiftFactor;

  // 根据当前动作选择抬腿因子
  switch (currentAction) {
    case FORWARD:
      coxaLiftFactor = FORWARD_COXA_LIFT_FACTORS[legIndex] * coxaMovementFactor;
      break;
    case BACKWARD:
      coxaLiftFactor = BACKWARD_COXA_LIFT_FACTORS[legIndex];
      break;
    case LEFT_TURN:
      coxaLiftFactor = LEFT_TURN_COXA_LIFT_FACTORS[legIndex] * coxaMovementFactor;
      break;
    case RIGHT_TURN:
      coxaLiftFactor = RIGHT_TURN_COXA_LIFT_FACTORS[legIndex];
      break;
    default:
      coxaLiftFactor = 0.1;
      break;
  }

  // 确定腿所属的三脚架组
  bool isGroup1 = (legIndex == 0 || legIndex == 2 || legIndex == 4);

  // 计算归一化相位（0 - 360度）
  int normalizedPhase = phase % 360;

  // 计算当前步态周期中的相位比例 (0.0 - 1.0)
  float phaseRatio = normalizedPhase / 360.0f;

  // 前进或后退
  if (currentAction == FORWARD || currentAction == BACKWARD) {
    // 确定腿是在摆动阶段还是支撑阶段
    bool isSwingLeg;
    if (isGroup1) {
      isSwingLeg = (normalizedPhase < 180);
    } else {
      isSwingLeg = (normalizedPhase >= 180);
    }

    // 归一化摆动相位（0 - 180度）
    int swingPhase = isSwingLeg ? normalizedPhase : (normalizedPhase - 180);

    // 使用正弦函数计算平滑的Coxa角度
    float angleOffset;
    if (isSwingLeg) {
      // 摆动阶段 - 向前移动
      angleOffset = COXA_SWING_FACTOR * stepLength * sin(radians(swingPhase));
    } else {
      // 支撑阶段 - 向后移动
      angleOffset = COXA_STANCE_FACTOR * stepLength * sin(radians(swingPhase + 180));
    }

    // 调整方向（前进或后退）
    coxaAngle += angleOffset * movementDirection;

    // 应用该腿的Coxa抬高度数因子
    if (isSwingLeg) {
      coxaAngle += coxaLiftFactor * stepHeight * sin(radians(swingPhase));
    }
  } else if (currentAction == LEFT_TURN || currentAction == RIGHT_TURN) {
    // 定义转向方向（左转为-1，右转为1）
    float turnDirection = (currentAction == LEFT_TURN) ? -1.0f : 1.0f;

    // 区分内外侧腿（外侧腿运动幅度更大）
    bool isOuterLeg = (currentAction == LEFT_TURN) ? (legIndex % 2 == 1) :  // 左转时右侧腿(2/4/6)为外侧
                        (legIndex % 2 == 0);                                // 右转时左侧腿(1/3/5)为外侧

    // 设置幅度系数（外侧腿1.5，内侧腿0.8）
    float amplitude = isOuterLeg ? 1.5f : 0.8f;

    // 基础转向角度（30°为基准值，可根据需要调整）
    float baseTurnAngle = 30.0f * turnDirection * amplitude;

    // 添加周期性微调（使转向更自然）
    float cycleVariation = sin(2 * PI * phaseRatio) * 5.0f;

    // 应用最终角度
    coxaAngle += baseTurnAngle + cycleVariation;
  }

  // 抬腿阶段逻辑
  bool isSwingPhase = (isGroup1 && normalizedPhase < 180) || (!isGroup1 && normalizedPhase >= 180);
  if (isSwingPhase) {
    int swingPhase = isGroup1 ? normalizedPhase : (normalizedPhase - 180);
    float liftFactor = sin(radians(swingPhase));
    coxaAngle += coxaLiftFactor * stepHeight * liftFactor;
  }

  // 角度安全限制
  coxaAngle = constrain(coxaAngle, MIN_COXA_ANGLE, MAX_COXA_ANGLE);
  return coxaAngle;
}

// 计算Femur角度 - 保持稳定不参与运动
int calculateFemurAngle(int legIndex, int phase) {
  // Femur舵机保持在初始位置，不参与运动
  return homeAngles[legIndex * 3 + 1] - initialHeightOffset;
}

int calculateTibiaAngle(int legIndex, int phase) {
  // 基础Tibia角度
  int tibiaAngle = homeAngles[legIndex * 3 + 2] + initialHeightOffset;

  // 超声波模式专用处理
  if (currentMode == ULTRASONIC_MODE) {
    float tibiaLiftFactor;

    // 根据当前动作选择抬腿因子
    switch (currentAction) {
      case ULTRASONIC_MOVING:
        tibiaLiftFactor = ULTRASONIC_FORWARD_TIBIA_LIFT_FACTORS[legIndex];
        break;
      case ULTRASONIC_TURNING:
        tibiaLiftFactor = ULTRASONIC_TURN_TIBIA_LIFT_FACTORS[legIndex];
        break;
      default:
        tibiaLiftFactor = (legIndex % 2 == 0) ? 1.0 : -1.0;
        break;
    }

    // 确定腿所属的三脚架组
    bool isGroup1 = (legIndex == 0 || legIndex == 2 || legIndex == 4);

    // 计算归一化相位（0-360度）
    int normalizedPhase = phase % 360;

    // 确定腿是在摆动阶段还是支撑阶段
    bool isSwingLeg;
    if (isGroup1) {
      isSwingLeg = (normalizedPhase < 180);
    } else {
      isSwingLeg = (normalizedPhase >= 180);
    }

    // 归一化摆动相位（0-180度）
    int swingPhase = isSwingLeg ? normalizedPhase : (normalizedPhase - 180);

    if (isSwingLeg) {
      // 使用正弦函数计算平滑的抬腿曲线
      float lift = sin(radians(swingPhase));
      tibiaAngle += stepHeight * lift * tibiaLiftFactor * 1.5;  // 增加抬腿幅度

      // 对于转向动作，根据内外侧腿调整抬腿高度
      if (currentAction == ULTRASONIC_TURNING) {
        bool isOuterLeg = (legIndex % 2 == 1);  // 左转时右侧腿(2/4/6)为外侧

        // 外侧腿抬得更高
        if (isOuterLeg) {
          tibiaAngle += 5;  // 额外增加抬腿高度
        }
      }
    } else {
      // 支撑阶段稍微降低身体
      tibiaAngle -= 3;
    }

    // 确保Tibia角度不超过安全范围
    tibiaAngle = constrain(tibiaAngle, 30, 150);
    return tibiaAngle;
  }

  // 以下是原有手动模式的计算逻辑（保持不变）
  float heightReduction = 1.0;
  float tibiaLiftFactor;

  // 根据当前动作选择抬腿因子
  switch (currentAction) {
    case FORWARD:
      tibiaLiftFactor = FORWARD_TIBIA_LIFT_FACTORS[legIndex] * heightReduction;
      break;
    case BACKWARD:
      tibiaLiftFactor = BACKWARD_TIBIA_LIFT_FACTORS[legIndex];
      break;
    case LEFT_TURN:
      tibiaLiftFactor = LEFT_TURN_TIBIA_LIFT_FACTORS[legIndex] * heightReduction;
      break;
    case RIGHT_TURN:
      tibiaLiftFactor = RIGHT_TURN_TIBIA_LIFT_FACTORS[legIndex];
      break;
    default:
      tibiaLiftFactor = (legIndex % 2 == 0) ? 1.5 : -1.5;
      break;
  }

  // 确定腿所属的三脚架组
  bool isGroup1 = (legIndex == 0 || legIndex == 2 || legIndex == 4);

  // 计算归一化相位（0-360度）
  int normalizedPhase = phase % 360;

  // 确定腿是在摆动阶段还是支撑阶段
  bool isSwingLeg;
  if (isGroup1) {
    isSwingLeg = (normalizedPhase < 180);
  } else {
    isSwingLeg = (normalizedPhase >= 180);
  }

  // 归一化摆动相位（0-180度）
  int swingPhase = isSwingLeg ? normalizedPhase : (normalizedPhase - 180);

  if (isSwingLeg) {
    // 使用正弦函数计算平滑的抬腿曲线
    float lift = sin(radians(swingPhase));
    tibiaAngle += stepHeight * lift * tibiaLiftFactor;

    // 对于转向动作，根据内外侧腿调整抬腿高度
    if (currentAction == LEFT_TURN || currentAction == RIGHT_TURN) {
      bool isOuterLeg = (currentAction == LEFT_TURN) ? (legIndex % 2 == 1) :  // 左转时右侧腿(2/4/6)为外侧
                          (legIndex % 2 == 0);                                // 右转时左侧腿(1/3/5)为外侧

      // 外侧腿抬得更高
      if (isOuterLeg) {
        tibiaAngle += 5;  // 额外增加抬腿高度
      }
    }
  } else {
    // 支撑阶段稍微降低身体
    tibiaAngle -= 3;
  }

  // 确保Tibia角度不超过安全范围
  tibiaAngle = constrain(tibiaAngle, 30, 150);
  return tibiaAngle;
}

// 平滑舵机角度变化
int smoothServoAngle(int currentAngle, int targetAngle) {
  // 使用平滑系数来平滑舵机角度变化
  return (int)(currentAngle * SMOOTHING_FACTOR + targetAngle * (1 - SMOOTHING_FACTOR));
}

// 计算两个动作之间的相位偏移，确保平滑过渡
int calculatePhaseOffset(RobotState lastAction, RobotState currentAction) {
  // 如果是从停止状态开始，不需要相位偏移
  if (lastAction == STOP) {
    return 0;
  }

  // 如果是前进和后退之间的转换，需要180度的相位偏移
  if ((lastAction == FORWARD && currentAction == BACKWARD) || (lastAction == BACKWARD && currentAction == FORWARD)) {
    return 180;
  }

  // 如果是左转和右转之间的转换，需要180度的相位偏移
  if ((lastAction == LEFT_TURN && currentAction == RIGHT_TURN) || (lastAction == RIGHT_TURN && currentAction == LEFT_TURN)) {
    return 180;
  }

  // 如果是前进/后退与左转/右转之间的转换，需要90度的相位偏移
  if ((lastAction == FORWARD || lastAction == BACKWARD) && (currentAction == LEFT_TURN || currentAction == RIGHT_TURN)) {
    return 90;
  }

  if ((lastAction == LEFT_TURN || lastAction == RIGHT_TURN) && (currentAction == FORWARD || currentAction == BACKWARD)) {
    return 270;
  }

  // 其他情况不需要相位偏移
  return 0;
}

// 检查是否可以安全转换状态
bool checkSafeToTransition() {
  // 禁止在关键相位过渡期间进行状态转换
  if (phaseTransitioning || criticalPhaseCounter > 0) {
    return false;
  }

  // 状态转换完成且系统稳定时才允许转换
  if (!transitioningState && stabilityCounter >= STABILITY_THRESHOLD) {
    return true;
  }

  return false;
}

// 检查当前相位是否是安全转换点
bool isSafeTransitionPhase() {
  // 安全相位点定义为0°、90°、180°和270°附近
  int safePhasePoints[4] = { 0, 90, 180, 270 };
  int tolerance = 10;  // 容差范围

  for (int i = 0; i < 4; i++) {
    if (abs(gaitPhase - safePhasePoints[i]) <= tolerance) {
      return true;
    }
  }

  return false;
}

// 三角步态控制
void tripodGait() {
  // 1. 运动方向确定（超声波转向复用左/右转方向修正）
  const int* currentDirection;
  if (currentAction == LEFT_TURN || 
      (currentAction == ULTRASONIC_TURNING && isTurningLeft)) {
    currentDirection = leftTurnServoDirection;
  } else if (currentAction == RIGHT_TURN || 
             (currentAction == ULTRASONIC_TURNING && !isTurningLeft)) {
    currentDirection = rightTurnServoDirection;
  } else {
    currentDirection = walkServoDirection;
  }

  // 2. 状态转换处理（严格隔离模式）
  static int lastAction = -1;
  static OperationMode lastMode = MANUAL_MODE;
  if (lastAction != currentAction || lastMode != currentMode) {
    gaitPhaseOffset = calculatePhaseOffset((RobotState)lastAction, currentAction);
    lastAction = currentAction;
    lastMode = currentMode;
    
    // 重置运动状态
    accumulatedPhase = 0;
    gaitPhase = 0;
    previousGaitPhase = 0;
    
    // 重置舵机角度记录
    for (int i = 0; i < 6; i++) {
      currentCoxaAngles[i] = homeAngles[i * 3];
      currentFemurAngles[i] = homeAngles[i * 3 + 1];
    }
  }

  // 3. 相位计算（两种模式共用）
  int effectivePhase = (accumulatedPhase + gaitPhaseOffset) % 360;
  accumulatedPhase += PHASE_INCREMENT;

  // 4. 六腿运动控制
  for (int legIndex = 0; legIndex < 6; legIndex++) {
    int coxaIndex = legIndex * 3;
    int femurIndex = legIndex * 3 + 1;
    int tibiaIndex = legIndex * 3 + 2;

    int coxaAngle, femurAngle, tibiaAngle;

    /*----- 核心修改开始（超声波模式复用手动算法）-----*/
    // 4.1 超声波模式完全复用手动算法
    if (currentMode == ULTRASONIC_MODE) {
      // 复用原有计算函数
      coxaAngle = calculateCoxaAngle(legIndex, effectivePhase);
      femurAngle = calculateFemurAngle(legIndex, effectivePhase);
      tibiaAngle = calculateTibiaAngle(legIndex, effectivePhase);

      // 超声波转向强化（保持方向一致性）
      if (currentAction == ULTRASONIC_TURNING) {
        // 确定内外侧腿（左转时2/4/6为外侧，右转时1/3/5为外侧）
        bool isOuterLeg = isTurningLeft ? (legIndex % 2 == 1) : (legIndex % 2 == 0);
        
        // 外侧腿增强20%幅度，内侧腿保持原样
        float boostFactor = isOuterLeg ? 1.2f : 1.0f;
        coxaAngle = homeAngles[coxaIndex] + 
                   (coxaAngle - homeAngles[coxaIndex]) * boostFactor;
      }
    } 
    // 4.2 手动模式保持原样
    else {
      coxaAngle = calculateCoxaAngle(legIndex, effectivePhase);
      femurAngle = calculateFemurAngle(legIndex, effectivePhase);
      tibiaAngle = calculateTibiaAngle(legIndex, effectivePhase);
    }
    /*----- 核心修改结束 -----*/

    // 4.3 平滑过渡（共用）
    coxaAngle = smoothServoAngle(currentCoxaAngles[legIndex], coxaAngle);
    currentCoxaAngles[legIndex] = coxaAngle;

    femurAngle = smoothServoAngle(currentFemurAngles[legIndex], femurAngle);
    currentFemurAngles[legIndex] = femurAngle;

    // 4.4 应用角度（带安全限制）
    servos[coxaIndex].write(constrain(
      homeAngles[coxaIndex] + (coxaAngle - homeAngles[coxaIndex]) * currentDirection[coxaIndex],
      MIN_COXA_ANGLE, MAX_COXA_ANGLE
    ));

    servos[femurIndex].write(constrain(
      homeAngles[femurIndex] + (femurAngle - homeAngles[femurIndex]) * currentDirection[femurIndex],
      40, 140
    ));

    servos[tibiaIndex].write(constrain(
      homeAngles[tibiaIndex] + (tibiaAngle - homeAngles[tibiaIndex]) * currentDirection[tibiaIndex],
      40, 140
    ));
  }

  // 5. 相位更新
  gaitPhase = accumulatedPhase % 360;
  cycleCompleted = (previousGaitPhase > gaitPhase);
  previousGaitPhase = gaitPhase;
}

// 测量超声波距离（简化输出）
float measureDistance() {
  const int SAMPLES = 5;  // 采样次数
  float distances[SAMPLES];

  for (int i = 0; i < SAMPLES; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long duration = pulseIn(ECHO_PIN, HIGH);
    distances[i] = duration * 0.034 / 2;
    delay(10);  // 防止连续测量干扰
  }

  // 冒泡排序代替 std::sort
  for (int i = 0; i < SAMPLES - 1; i++) {
    for (int j = 0; j < SAMPLES - i - 1; j++) {
      if (distances[j] > distances[j + 1]) {
        float temp = distances[j];
        distances[j] = distances[j + 1];
        distances[j + 1] = temp;
      }
    }
  }

  float medianDistance = distances[SAMPLES / 2];  // 中值

  // 低通滤波（平滑）
  static float filteredDistance = medianDistance;
  filteredDistance = filteredDistance * 0.7 + medianDistance * 0.3;

  // 限制有效范围（0-150cm）
  if (filteredDistance <= 0 || filteredDistance > 150) {
    filteredDistance = lastValidDistance;
  } else {
    lastValidDistance = filteredDistance;
  }

  return filteredDistance;
}

// 读取环境传感器数据（保留主要输出）
void readEnvironmentSensors() {
  static unsigned long lastReadTime = 0;
  if (currentTime - lastReadTime < 2000) {
    return;
  }
  lastReadTime = currentTime;

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("无法读取DHT11传感器数据!");
  } else {
    Serial.print("环境温度: ");
    Serial.print(temperature);
    Serial.print(" °C, 湿度: ");
    Serial.print(humidity);
    Serial.println(" %");
  }

  int mq9Value = digitalRead(MQ9_DO_PIN);
  Serial.print("MQ-9烟雾传感器: ");
  Serial.println(mq9Value ? "正常" : "检测到烟雾!");

  // 只有在手动模式下才触发蜂鸣器
  if (currentMode == MANUAL_MODE) {
    // 检测到烟雾时触发蜂鸣器
    if (mq9Value == LOW && !buzzerActive) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerStartTime = currentTime;
      buzzerActive = true;
      Serial.println("检测到可燃气体，蜂鸣器启动!");
    }
  }

  int pirValue = digitalRead(PIR_PIN);
  Serial.print("人体检测传感器: ");
  Serial.println(pirValue ? "检测到移动" : "无移动");

  // 只有在手动模式下才触发蜂鸣器
  if (currentMode == MANUAL_MODE) {
    // 检测到人体移动时触发蜂鸣器
    if (pirValue == HIGH && !buzzerActive) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerStartTime = currentTime;
      buzzerActive = true;
      Serial.println("检测到人体移动，蜂鸣器启动!");
    }
  }
}

// 处理红外遥控器命令（重构超声波模式处理）
void handleIRCommand(unsigned long command) {
  unsigned long currentIRTime = millis();

  // 消抖处理
  if (currentIRTime - lastValidIRTime < IR_DEBOUNCE_TIME && command == lastIRCommand) {
    irrecv.resume();
    return;
  }

  // 命令过滤
  bool validCommand = false;
  switch (command) {
    case 0xFF38C7:  // OK键
    case 0xFFA25D:  // 1键
    case 0xFFB04F:  // #键
    case 0xFF10EF:  // 左键
    case 0xFF18E7:  // 上键
    case 0xFF5AA5:  // 右键
    case 0xFF4AB5:  // 下键
      validCommand = true;
      break;
    default:
      Serial.print("忽略未知命令: 0x");
      Serial.println(command, HEX);
      irrecv.resume();
      return;
  }

  if (validCommand) {
    lastIRCommand = command;
    lastValidIRTime = currentIRTime;

    // 模式切换特殊处理
    if (command == 0xFFA25D || command == 0xFFB04F) {
      processModeSwitchCommand(command);
    } else {
      // 确保系统已初始化
      if (!systemInitialized) {
        smoothInitialize();
      }

      // 确保在HOME位置
      if (currentAction == INITIALIZING) {
        currentAction = STOP;
        stand();
      }

      // 处理命令
      if (currentMode == MANUAL_MODE) {
        processManualModeCommand(command);
      } else {
        if (command == 0xFF38C7) {  // OK键
          processUltrasonicModeCommand(command);
        }
      }
    }
  }

  irrecv.resume();
}

// 处理模式切换命令
void processModeSwitchCommand(unsigned long command) {
  switch (command) {
    case 0xFFA25D:  // 1键 - 进入超声波模式
      if (currentMode == MANUAL_MODE) {
        currentMode = ULTRASONIC_MODE;
        currentAction = ULTRASONIC_INIT;
        ultrasonicInitialized = false;
        isTurningLeft = false;
        Serial.println("进入超声波模式");
      }
      break;

    case 0xFFB04F:  // #键 - 退出超声波模式
      if (currentMode == ULTRASONIC_MODE) {
        currentMode = MANUAL_MODE;
        currentAction = RETURNING_HOME;
        returnHomeStartTime = currentTime;  // 记录返回HOME开始时间
        Serial.println("退出超声波模式，返回手动模式");

        // 立即设置所有舵机向HOME位置移动
        for (int i = 0; i < SERVO_NUM; i++) {
          servos[i].write(homeAngles[i]);
        }
      }
      break;
  }
}

// 处理手动模式命令
void processManualModeCommand(unsigned long command) {
  switch (command) {
    case 0xFF38C7:  // OK键 - 停止
      if (currentAction != STOP) {
        currentAction = STOP;
        movementDirection = 1;  // 重置方向
        for (int i = 0; i < SERVO_NUM; i++) {
          servos[i].write(homeAngles[i]);
        }
        Serial.println("停止");
      }
      break;

    case 0xFF10EF:  // 左键 - 左转
      if (currentAction != LEFT_TURN) {
        currentAction = LEFT_TURN;
        movementDirection = 1;  // 重置方向
        setLeftTurnParams(speed);
        gaitPhase = 0;
        Serial.println("左转");
      }
      break;

    case 0xFF18E7:  // 上键 - 前进
      if (currentAction != FORWARD) {
        currentAction = FORWARD;
        movementDirection = 1;  // 明确设置为前进
        setForwardParams(speed);
        gaitPhase = 0;
        Serial.println("前进");
      }
      break;

    case 0xFF5AA5:  // 右键 - 右转
      if (currentAction != RIGHT_TURN) {
        currentAction = RIGHT_TURN;
        movementDirection = 1;  // 重置方向
        setRightTurnParams(speed);
        gaitPhase = 0;
        Serial.println("右转");
      }
      break;

    case 0xFF4AB5:  // 下键 - 后退
      if (currentAction != BACKWARD) {
        currentAction = BACKWARD;
        movementDirection = -1;  // 明确设置为后退
        setBackwardParams(speed);
        gaitPhase = 0;
        Serial.println("后退");
      }
      break;
  }
}

// 处理超声波模式命令
void processUltrasonicModeCommand(unsigned long command) {
  if (command == 0xFF38C7) {  // OK键
    currentAction = RETURNING_HOME;
    returnHomeStartTime = currentTime;
    Serial.println("超声波模式: 停止并返回HOME位置");
  }
}

// 重置运动状态
void resetMovementState() {
  gaitPhase = 0;
  accumulatedPhase = 0;
  previousGaitPhase = 0;
  movementCycleCount = 0;
  commandLocked = false;
  transitioningState = false;
  stabilityCounter = 0;
  forcePhaseReset = true;
}

void initializeUltrasonicMode() {
  if (!ultrasonicInitialized) {
    setUltrasonicForwardParams();
    returnToHome();  // 确保从HOME位置开始
    ultrasonicInitialized = true;
    Serial.println("超声波模式初始化完成");
  }
  currentAction = ULTRASONIC_MOVING;
}

// 超声波模式主逻辑处理函数
void handleUltrasonicMode() {
  static bool firstRun = true;

  // 初始化处理
  if (firstRun) {
    ultrasonicInitialized = false;
    firstRun = false;
  }

  switch (currentAction) {
    case ULTRASONIC_INIT:
      initializeUltrasonicMode();
      break;

    case ULTRASONIC_MOVING:
      handleUltrasonicMoving();
      break;

    case ULTRASONIC_TURNING:
      handleUltrasonicTurning();
      break;

    default:
      currentAction = ULTRASONIC_INIT;
      break;
  }
}

// 超声波前进状态处理
void handleUltrasonicMoving() {
    // 定期测量距离
    if (currentTime - lastUltrasonicCheck > ULTRASONIC_CHECK_INTERVAL) {
        float distance = measureDistance();
        
        if (distance < OBSTACLE_DISTANCE && currentAction != ULTRASONIC_TURNING) {
            // 开始转向的调用
            currentAction = ULTRASONIC_TURNING;
            isTurningLeft = true;
            ultrasonicActionStartTime = currentTime; // 记录转向开始时间
            Serial.println("检测到障碍物，开始左转");
            
            // 重置相位参数确保平滑
            accumulatedPhase = 0;
            gaitPhase = 0;
            return; // 立即进入转向状态
        }
        lastUltrasonicCheck = currentTime;
    }

  // 正常前进
  if ((currentTime - previousTime) >= frameTimeMs) {
    previousTime = currentTime;
    tripodGait();
  }
}

// 超声波转向状态处理
void handleUltrasonicTurning() {
    // 持续转向动作
    if ((currentTime - previousTime) >= frameTimeMs) {
        previousTime = currentTime;
        tripodGait();
    }

    // 转向完成检测（移除方向反转逻辑）
    if (currentTime - ultrasonicActionStartTime > ULTRASONIC_TURN_DURATION) {
        float distance = measureDistance();
        
        if (distance > OBSTACLE_DISTANCE + 5) { // 保持5cm余量
            // 平滑过渡到前进
            setUltrasonicForwardParams();
            currentAction = ULTRASONIC_MOVING;
            gaitPhase = 180; // 从支撑相位开始
            Serial.println("转向完成→前进");
        } else {
            // 继续同方向转向（关键修改：移除!isTurningLeft）
            ultrasonicActionStartTime = currentTime;
            Serial.println("继续同向转向");
        }
    }
}

// 添加蜂鸣器控制函数
void controlBuzzer() {
  if (buzzerActive) {
    if (currentTime - buzzerStartTime >= BUZZER_DURATION) {
      digitalWrite(BUZZER_PIN, LOW);  // 关闭蜂鸣器
      buzzerActive = false;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("六足机器人控制系统启动中...");

  // 初始化蜂鸣器引脚
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // 初始化红外接收
  irrecv.enableIRIn();
#ifdef IR_PIN_PULLUP
  pinMode(RECV_PIN, INPUT_PULLUP);
#endif

  currentTime = millis();
  previousTime = currentTime;
  lastCommandTime = currentTime;
  commandStartTime = currentTime;
  lastSerialOutputTime = currentTime;
  lastSensorReadTime = currentTime;

  // 初始化状态变量
  systemInitialized = false;
  stanceLegSettled = false;
  lastCommandWasStop = false;
  transitioningState = false;
  stabilityCounter = 0;
  resetProtection = 0;
  irErrorCount = 0;
  irErrorDetected = false;
  commandLocked = false;
  turnCyclesCount = 0;  // 初始化转向周期计数器

  // 初始化舵机角度记录
  for (int i = 0; i < SERVO_NUM / 3; i++) {
    currentCoxaAngles[i] = homeAngles[i * 3];
  }

  // 看门狗初始化
  if (!WATCHDOG_DISABLED) {
    wdt_disable();
    wdt_enable(WDTO_4S);
  }

  Serial.println("系统初始化完成，等待命令...");
}

void loop() {
  // 看门狗复位
  //if (!WATCHDOG_DISABLED) wdt_reset();

  currentTime = millis();

  // 处理红外信号
  if (irrecv.decode(&results)) {
    handleIRCommand(results.value);
    irrecv.resume();
  }

  // 每2秒读取一次环境传感器（仅在手动模式下）
  if (currentMode == MANUAL_MODE && currentTime - lastSensorReadTime > 2000) {
    readEnvironmentSensors();
    lastSensorReadTime = currentTime;
  }

  controlBuzzer();

  // 模式处理
  if (currentMode == MANUAL_MODE) {
    switch (currentAction) {
      case INITIALIZING:
        smoothInitialize();
        break;
      case RETURNING_HOME:
        if (returnToHome()) {
          // 添加500ms稳定时间
          if (currentTime - returnHomeStartTime > 500) {
            currentAction = ULTRASONIC_MOVING;
            lastUltrasonicCheck = currentTime;
            Serial.println("HOME位置稳定，重新开始探测");
          }
        }
        break;
      case STOP:
        stand();  // 保持站立姿态
        break;
      case FORWARD:
      case BACKWARD:
      case LEFT_TURN:
      case RIGHT_TURN:
        if ((currentTime - previousTime) >= frameTimeMs) {
          previousTime = currentTime;
          tripodGait();
        }
        break;
      default:
        currentAction = STOP;
        stand();
        break;
    }
  } else {  // 超声波模式
    handleUltrasonicMode();
  }
  // 系统保护机制
  resetProtection++;
  if (resetProtection > 10000) {
    resetProtection = 0;
    if (!WATCHDOG_DISABLED) {
      wdt_disable();
      delay(100);
      wdt_enable(WDTO_15MS);
    }
  }

  // 模式切换后的延迟处理
  if (needDelayAfterModeSwitch && currentTime - lastModeSwitchTime > 500) {
    needDelayAfterModeSwitch = false;
  }

  // 确保系统初始化
  if (!systemInitialized) {
    smoothInitialize();
  }
}