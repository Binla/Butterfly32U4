// 250507 Version14 - Sensitivity & Bias Optimization
// Optimized by Antigravity AI - Bias ±10deg, Steering ±40deg, Pitch ±20deg
#include <Arduino.h>         // 引用 Arduino 核心庫
#include <DSMRX.h>           // 引用 DSMX 接收機庫
#include <SoftwareSerial.h>  // 引用 軟串口庫 用於調試

// --- 硬體配置 ---
const int PIN_SERVO_L = 9;   // 左舵機引腳 (必須為 9，用於 Timer 1 硬體 PWM)
const int PIN_SERVO_R = 10;  // 右舵機引腳 (必須為 10，用於 Timer 1 硬體 PWM)
const int PIN_DEBUG_RX = 7;  // 調試串口接收引腳
const int PIN_DEBUG_TX = 8;  // 調試串口發送引腳

const int PWM_MIN = 1000;    // 標準 PWM 最小值
const int PWM_MAX = 2000;    // 標準 PWM 最大值
const int PWM_CENTER = 1500; // 標準 PWM 中立點
const int SERVO_LIMIT_MIN = 500;   // 舵機物理極限最小值 (擴展至 500us)
const int SERVO_LIMIT_MAX = 2500;  // 舵機物理極限最大值 (擴展至 2500us)

const int SAFETY_THRESHOLD = 1600; // 安全開關閾值 (Ch6)
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

// --- 當前接收機快取 (用於超採樣計算) ---
volatile int rxThrottle = 1000;
volatile int rxAileron  = 1500;
volatile int rxElevator = 1500;
volatile int rxAilTrim  = 1500; // 新增：副翼基準點調整 (Ch4)
volatile int rxEleTrim  = 1500; // 新增：升降舵基準點調整 (Ch5)
volatile int rxSafetyCh = 1000;

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

void setup() {
  Serial.begin(115200); 
  debug.begin(38400);   
  debug.println("Butterfly Ornithopter - v8 8.4V Extreme-Mode Started");

  setupPWM333Hz(); 

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
    rxAilTrim   = ch[3]; // 讀取 Ch4 用於副翼 Trim
    rxEleTrim   = ch[4]; // 讀取 Ch5 用於升降舵 Trim
    rxSafetyCh  = ch[5]; 
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
    outL = WING_STOP_L;
    outR = WING_STOP_R;
  } else {
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
      // --- 正常飛行邏輯 (基準點可調版) ---
      int ailInput = (rxAileron - PWM_CENTER);      
      int eleInput = (rxElevator - PWM_CENTER);   
      int ailOffset = (int)((rxAilTrim - PWM_CENTER) * 0.22);  // Ch4 靜態偏置限縮 (±10deg)
      int eleOffset = (int)((rxEleTrim - PWM_CENTER) * 0.22);  // Ch5 靜態偏置限縮 (±10deg)

      // 1. 中立點計算：由 Pitch 控制 (限 ±20°) 與 Bias 修正
      int dynamicEle = (int)(eleInput * 0.44); // 動態升降限縮 (±20deg / ±500單位)
      int totalEle = dynamicEle + eleOffset;
      int centerL = PWM_CENTER + totalEle + ailOffset; 
      int centerR = PWM_CENTER - totalEle + ailOffset; // 右側俯仰反向，Roll Bias 同向 (鏡像安裝)

      if (rxThrottle > THROTTLE_ARM_PWM) {
        float tScale = constrain((rxThrottle - THROTTLE_ARM_PWM) / (float)(PWM_MAX - THROTTLE_ARM_PWM), 0.0, 1.0);
        float currentFreq = LIMIT_FREQ_MIN + (tScale * (LIMIT_FREQ_MAX - LIMIT_FREQ_MIN));
        
        // 分段權衡模式：
        // 0%-50% 油門: 振幅 90° -> 75°
        // 50%-100% 油門: 振幅 75° -> 40°
        float currentAmp;
        if (tScale < 0.5) {
          float tSub = tScale * 2.0; 
          currentAmp = AMP_MAX_US - (tSub * (AMP_MAX_US - AMP_MID_US));
        } else {
          float tSub = (tScale - 0.5) * 2.0;
          currentAmp = AMP_MID_US - (tSub * (AMP_MID_US - AMP_MIN_US));
        }
        
        cyclePhase += currentFreq * dt * TWO_PI;
        if (cyclePhase > TWO_PI) cyclePhase -= TWO_PI; 

        // 2. 振幅計算：由 Throttle 基礎值與 Roll 差動值決定
        float diffAmp = ailInput * 0.88; // 差動振幅加強 (±40deg / ±500單位)
        float ampL = currentAmp - diffAmp;
        float ampR = currentAmp + diffAmp;

        float wave = sin(cyclePhase);
        outL = centerL + (int)(wave * ampL);
        outR = centerR - (int)(wave * ampR); // 右側相位反向
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
}

void serialEvent() {
  while (Serial.available()) {
    rx.handleSerialEvent(Serial.read(), micros());
  }
}