// 250511 Version15 - Setup Mode & Rudder Implementation
// Optimized by Antigravity AI
// 
// --- 遙控器通道配置 (Channel Mapping) ---
// Ch1 (ch[0]): 油門 (Throttle) - 控制拍動頻率 (Hz)
// Ch2 (ch[1]): 副翼 (Aileron)  - 控制左右差動振幅 (Roll)，設定模式下調整左舵機微調
// Ch3 (ch[2]): 升降 (Elevator) - 控制同向偏置 (Pitch)，設定模式下調整右舵機微調
// Ch4 (ch[3]): 方向 (Rudder)   - 控制反向偏置 (Yaw/航向)，向左打到底儲存並退出設定
// Ch5 (ch[4]): 未使用 (空置)
// Ch6 (ch[5]): 振幅 (Amplitude)- 使用旋鈕控制基礎拍動角度 (40~90度)
// Ch7 (ch[6]): 安全 (Safety)   - 安全開關 (Arm/Disarm)，上鎖狀態下可進入設定模式
#include <Arduino.h>         // 引用 Arduino 核心庫
#include <DSMRX.h>           // 引用 DSMX 接收機庫
#include <SoftwareSerial.h>  // 引用 軟串口庫 用於調試
#include <EEPROM.h>          // 引用 EEPROM 庫 用於保存微調

// --- 硬體配置 ---
const int PIN_SERVO_L = 9;   // 左舵機引腳 (必須為 9，用於 Timer 1 硬體 PWM)
const int PIN_SERVO_R = 10;  // 右舵機引腳 (必須為 10，用於 Timer 1 硬體 PWM)
const int PIN_DEBUG_RX = 7;  // 調試串口接收引腳
const int PIN_DEBUG_TX = 8;  // 調試串口發送引腳
const int PIN_LED = 13;      // LED 引腳 (PD13/D13)

const int PWM_MIN = 1000;    // 標準 PWM 最小值
const int PWM_MAX = 2000;    // 標準 PWM 最大值
const int PWM_CENTER = 1500; // 標準 PWM 中立點
const int SERVO_LIMIT_MIN = 500;   // 舵機物理極限最小值 (擴展至 500us)
const int SERVO_LIMIT_MAX = 2500;  // 舵機物理極限最大值 (擴展至 2500us)

const int SAFETY_THRESHOLD = 1600; // 安全開關閾值 (Ch7)
const int THROTTLE_ARM_PWM = 1080; // 油門解鎖閾值
const int WING_STOP_L = 2100;    // 左舵機上揚位
const int WING_STOP_R = 900;     // 右舵機上揚位 (鏡像)

// --- 撲翼機動力限制 (PTK 7432 @ 8.4V 極限匹配) ---
const float LIMIT_FREQ_MIN = 1.56;  // 最低拍動頻率 (Hz)
const float LIMIT_FREQ_MAX = 3.125; // 最高拍動頻率 (Hz)
const float US_PER_DEGREE = 11.11;  // 1000us/90deg 換算
const float AMP_MIN_US = 40.0 * US_PER_DEGREE; // 最低振幅 40度 (約 444us)
const float AMP_MID_US = 75.0 * US_PER_DEGREE; // 中間點振幅 75度 (約 833us)
const float AMP_MAX_US = 90.0 * US_PER_DEGREE; // 最高振幅 90度 (約 1000us)

// --- 全域對象 ---
DSM2048 rx; // DSMX 接收機
SoftwareSerial debug(PIN_DEBUG_RX, PIN_DEBUG_TX); // 調試串口

// --- 狀態變數 ---
unsigned long lastMicros = 0;  
float cyclePhase = 0.0;        
bool isSystemArmed = false;    
bool isInSetupMode = false;    // 新增：進入設定模式標記

int savedOffsetL = 0;          // 新增：左舵機存儲偏置
int savedOffsetR = 0;          // 新增：右舵機存儲偏置
unsigned long lastSetupUpdate = 0; // 用於微調頻率限制

// --- LED 控制相關 ---
unsigned long ledTimer = 0;
int ledSubState = 0;
bool isAdjustingL = false;
bool isAdjustingR = false;
bool aileronPushed = false;   // 新增：防連發標記
bool elevatorPushed = false;  // 新增：防連發標記

// --- 當前接收機快取 (用於超採樣計算) ---
volatile int rxThrottle = 1000;
volatile int rxAileron  = 1500;
volatile int rxElevator = 1500;
volatile int rxRudder   = 1500; // 改名：Ch4 現在作為 Rudder 控制
volatile int rxAmpCh    = 1500; // 讀取 Ch6 用於振幅控制
volatile int rxSafetyCh = 1000; // 安全開關 (Ch7)

// --- PWM 硬體定時器控制 (333Hz) ---
void setupPWM333Hz() {
  pinMode(PIN_SERVO_L, OUTPUT); 
  pinMode(PIN_SERVO_R, OUTPUT); 
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); 
  ICR1 = 6000; // 3ms 週期
}

void writePWM(int pin, int us) {
  us = constrain(us, SERVO_LIMIT_MIN, SERVO_LIMIT_MAX); 
  int counts = us * 2; 
  if (pin == PIN_SERVO_L) OCR1A = counts;
  else if (pin == PIN_SERVO_R) OCR1B = counts;
}

// --- LED 邏輯函數 ---
void updateLED() {
  unsigned long now = millis();
  float ampScale = constrain((rxAmpCh - 1000) / 1000.0, 0.0, 1.0);

  if (isInSetupMode) {
    // 設定模式樣式
    int pulseCount = 2; // 預設 2 短亮
    if (isAdjustingL) pulseCount = 3;
    else if (isAdjustingR) pulseCount = 4;

    unsigned long cycleTime = 1000; 
    unsigned long localTime = now % cycleTime;
    
    if (localTime < 400) { // 1 長暗 (400ms)
      digitalWrite(PIN_LED, LOW);
    } else {
      // 剩餘 600ms 用於短亮
      unsigned long pulseTime = (localTime - 400) % 150;
      int currentPulse = (localTime - 400) / 150;
      if (currentPulse < pulseCount && pulseTime < 75) {
        digitalWrite(PIN_LED, HIGH);
      } else {
        digitalWrite(PIN_LED, LOW);
      }
    }
  } 
  else if (!isSystemArmed) {
    // 未解鎖：長亮
    digitalWrite(PIN_LED, HIGH);
  } 
  else {
    // 已解鎖
    if (rxThrottle > THROTTLE_ARM_PWM) {
      // 啟動後閃爍：頻率隨振幅變化 (3Hz ~ 15Hz)
      float flashFreq = 3.0 + ampScale * 12.0; 
      unsigned long flashPeriod = 1000.0 / flashFreq;
      if ((now % flashPeriod) < (flashPeriod / 2)) {
        digitalWrite(PIN_LED, HIGH);
      } else {
        digitalWrite(PIN_LED, LOW);
      }
    } else {
      // 解鎖未啟動：1秒呼吸燈
      // 由於 Pin 13 可能不支持硬體 PWM，這裡使用軟體模擬或簡單快速閃爍
      // 為了兼容性，我們使用 1Hz 慢閃模擬呼吸感
      if ((now % 1000) < 500) {
        digitalWrite(PIN_LED, HIGH);
      } else {
        digitalWrite(PIN_LED, LOW);
      }
    }
  }
}

void setup() {
  Serial.begin(115200); 
  debug.begin(38400);   
  debug.println("Butterfly Ornithopter - v15 Setup Mode Ready");

  // 從 EEPROM 讀取微調值 (地址 0 和 2)
  EEPROM.get(0, savedOffsetL);
  EEPROM.get(2, savedOffsetR);
  // 合法性檢查，防止讀取到非法數據 (初次運行)
  if (abs(savedOffsetL) > 400) savedOffsetL = 0;
  if (abs(savedOffsetR) > 400) savedOffsetR = 0;

  setupPWM333Hz(); 
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH); // 開機長亮

  delay(2000); 
  lastMicros = micros(); 
}

void loop() {
  // 1. 異步檢查接收機數據
  if (rx.gotNewFrame()) {
    uint16_t ch[8]; 
    rx.getChannelValues(ch, 8); 
    rxThrottle  = ch[0]; 
    rxAileron   = ch[1]; 
    rxElevator  = ch[2]; 
    rxRudder    = ch[3]; // Ch4 改為 Rudder 控制
    rxAmpCh     = ch[5]; // 讀取 Ch6 用於振幅控制
    rxSafetyCh  = ch[6]; // 讀取 Ch7 用於安全開關
  }

  // 2. 超採樣時間步進
  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastMicros) / 1000000.0;
  lastMicros = currentMicros;
  if (dt < 0 || dt > 0.1) dt = 0; 

  // 3. 輸出位置初始化
  int outL = WING_STOP_L; 
  int outR = WING_STOP_R; 

  // 4. 安全狀態機邏輯
  if (rxSafetyCh <= SAFETY_THRESHOLD) {
    isSystemArmed = false; 
    
    // --- 進入設定模式邏輯 ---
    // 當 Safety OFF (上鎖) 且 Rudder & Throttle 在右下角
    if (rxThrottle < 1100 && rxRudder > 1900) {
      if (!isInSetupMode) {
        isInSetupMode = true;
        debug.println("Entering Setup Mode...");
      }
    }

    if (isInSetupMode) {
      isAdjustingL = false;
      isAdjustingR = false;

      // 1. Aileron 左右調整左邊舵機 (Ch1) - 打到最大/最小調整 1度 (11us)
      if (rxAileron > 1900) {
        if (!aileronPushed) {
          savedOffsetL += 11; 
          aileronPushed = true;
          isAdjustingL = true;
        }
      } else if (rxAileron < 1100) {
        if (!aileronPushed) {
          savedOffsetL -= 11;
          aileronPushed = true;
          isAdjustingL = true;
        }
      } else if (rxAileron > 1400 && rxAileron < 1600) {
        aileronPushed = false; // 回中後重置，允許下一次調整
      }
      
      // 2. Elevator 上下調整右邊舵機 (Ch2) - 打到最大/最小調整 1度 (11us)
      if (rxElevator > 1900) {
        if (!elevatorPushed) {
          savedOffsetR += 11;
          elevatorPushed = true;
          isAdjustingR = true;
        }
      } else if (rxElevator < 1100) {
        if (!elevatorPushed) {
          savedOffsetR -= 11;
          elevatorPushed = true;
          isAdjustingR = true;
        }
      } else if (rxElevator > 1400 && rxElevator < 1600) {
        elevatorPushed = false; // 回中後重置
      }
      
      savedOffsetL = constrain(savedOffsetL, -500, 500);
      savedOffsetR = constrain(savedOffsetR, -500, 500);

      // 3. 設定模式下輸出當前位置以便觀察
      outL = PWM_CENTER + savedOffsetL;
      outR = PWM_CENTER + savedOffsetR;

      // 4. Rudder 往左打到底儲存並退出 (Ch4)
      if (rxRudder < 1100) {
        EEPROM.put(0, savedOffsetL);
        EEPROM.put(2, savedOffsetR);
        isInSetupMode = false;
        debug.println("Settings Saved. Exiting Setup Mode.");

        // LED 保存反饋：暗1秒 -> 亮2秒 -> 暗1秒
        digitalWrite(PIN_LED, LOW);
        delay(1000);
        digitalWrite(PIN_LED, HIGH);
        delay(2000);
        digitalWrite(PIN_LED, LOW);
        delay(1000);
      }
    } else {
      outL = WING_STOP_L;
      outR = WING_STOP_R;
    }
  } else {
    isInSetupMode = false; // 解鎖時自動退出設定模式
    if (!isSystemArmed) {
      if (rxThrottle < THROTTLE_ARM_PWM + 20) {
        isSystemArmed = true; 
        debug.println("System ARMED - 8.4V High Power Ready");
      } else {
        outL = WING_STOP_L;
        outR = WING_STOP_R;
      }
    }

    if (isSystemArmed) {
      // --- 正常飛行邏輯 (含 Rudder 控制與 Saved Offsets) ---
      int ailInput = (rxAileron - PWM_CENTER);      
      int eleInput = (rxElevator - PWM_CENTER);   
      int rudInput = (rxRudder - PWM_CENTER);   
      
      // 1. 中立點計算：
      // Pitch (Elevator) 現在設為同向控制 (Common Bias)
      // Yaw (Rudder) 設為反向控制 (Differential Bias)
      int commonPitch = (int)(eleInput * 0.44); 
      int diffYaw = (int)(rudInput * 0.44);

      int centerL = PWM_CENTER + savedOffsetL + commonPitch + diffYaw; 
      int centerR = PWM_CENTER + savedOffsetR - commonPitch + diffYaw; 
      // 注意：根據機械結構，Pitch 同向或反向可能需要調整。
      // 這裡假設 L+, R- 是向上抬升。

      if (rxThrottle > THROTTLE_ARM_PWM) {
        float tScale = constrain((rxThrottle - THROTTLE_ARM_PWM) / (float)(PWM_MAX - THROTTLE_ARM_PWM), 0.0, 1.0);
        float currentFreq = LIMIT_FREQ_MIN + (tScale * (LIMIT_FREQ_MAX - LIMIT_FREQ_MIN));
        
        float ampScale = constrain((rxAmpCh - 1000) / 1000.0, 0.0, 1.0);
        float currentAmp = AMP_MIN_US + (ampScale * (AMP_MAX_US - AMP_MIN_US));
        
        cyclePhase += currentFreq * dt * TWO_PI;
        if (cyclePhase > TWO_PI) cyclePhase -= TWO_PI; 

        // 2. 差動振幅 (Roll) - 增加安全約束防止相位反轉
        float diffAmp = ailInput * 0.88; 
        float ampL = currentAmp - diffAmp;
        float ampR = currentAmp + diffAmp;

        // 強制振幅不得為負值，防止左右翅膀進入反相位（剪刀差）導致機體翻轉
        if (ampL < 0.0) ampL = 0.0;
        if (ampR < 0.0) ampR = 0.0;

        float wave = sin(cyclePhase);
        outL = centerL + (int)(wave * ampL);
        outR = centerR - (int)(wave * ampR); 
      } else {
        cyclePhase = 0.0;
        outL = centerL;
        outR = centerR;
      }
    }
  }

  // 5. 輸出至硬體 PWM
  writePWM(PIN_SERVO_L, outL);
  writePWM(PIN_SERVO_R, outR);

  // 6. 更新 LED 狀態
  updateLED();
}

void serialEvent() {
  while (Serial.available()) {
    rx.handleSerialEvent(Serial.read(), micros());
  }
}