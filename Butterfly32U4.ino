// 250511 Version15 - Setup Mode & Rudder Implementation
// Optimized by Antigravity AI

// --- 遙控器通道配置 (Channel Mapping / プロポのチャンネル設定) ---
// Ch1: 油門 (Throttle / スロットル) - Flapping Frequency (Hz) / 拍動周波数
// Ch2: 副翼 (Aileron / エルロン) - Roll / Trim (Differential: one side high, one side low) / 左右差動微調
// Ch3: 升降 (Elevator / エレベーター) - Pitch / Trim (Common: both sides together) / 兩邊同步高低微調
// Ch4: 方向 (Rudder / ラダー) - Yaw / Setup Menu / ヨー、設定モード切替・保存
// Ch5: 未使用 (Unused / 未使用)
// Ch6: 振幅 (Amplitude / 振幅) - Flapping Angle (40~90°) / 拍動角度
// Ch7: 安全 (Safety / セーフティ) - Arm/Disarm Switch / アームスイッチ
// 
// --- LED 狀態指示 (LED Status / LED ステータス) ---
// 1. 常亮 (Solid / 点灯): 未解鎖 (Disarmed / 未解除)
// 2. 慢閃 (Slow Blink / ゆっくり点滅): 已解鎖，油門關閉 (Armed, Idle / 解除・待機中)
// 3. 快閃 (Fast Blink / 速い点滅): 飛行中 (Flying / 飛行中)
// 4. 設定模式 (Setup Mode / 設定モード): (Rudder Right to cycle, Left hold 3s to save)
//    - 1.1 (1L 3S): 雙舵微調 (Servo Trim - 升降同向, 副翼反向)
//    - 2.1 (2L 2S): 正反向 (Servo Reverse / サーボ正逆)
// 5. 保存成功 (Saved / 保存成功): Blink pattern / 点滅確認

#include <Arduino.h>         // 引用 Arduino 核心庫 (Arduino Core Library)
#include <DSMRX.h>           // 引用 DSMX 接收機庫 (DSMX Receiver Library)
#include <SoftwareSerial.h>  // 引用 軟串口庫 用於調試 (SoftwareSerial for Debugging)
#include <EEPROM.h>          // 引用 EEPROM 庫 用於保存微調 (EEPROM for saving trims)

// --- 硬體配置 (Hardware Configuration) ---
const int PIN_SERVO_L = 9;   // 左舵機引腳 (Left Servo Pin - Timer 1 Hardware PWM)
const int PIN_SERVO_R = 10;  // 右舵機引腳 (Right Servo Pin - Timer 1 Hardware PWM)
const int PIN_DEBUG_RX = 8;  // 調試串口接收引腳 (Debug Serial RX - Swapped to Pin 8 for ATmega32U4 SoftwareSerial compatibility)
const int PIN_DEBUG_TX = 7;  // 調試串口發送引腳 (Debug Serial TX - Swapped to Pin 7)
const int PIN_LED = 5;       // LED 引腳 (Status LED Pin - Pro Micro lacks Pin 13, changed to D5)

const int PWM_MIN = 1000;    // 標準 PWM 最小值 (Standard PWM Minimum)
const int PWM_MAX = 2000;    // 標準 PWM 最大值 (Standard PWM Maximum)
const int PWM_CENTER = 1500; // 標準 PWM 中立點 (Standard PWM Center)
const int SERVO_LIMIT_MIN = 500;   // 舵機物理極限最小值 (Servo Physical Limit Min)
const int SERVO_LIMIT_MAX = 2500;  // 舵機物理極限最大值 (Servo Physical Limit Max)

const int SAFETY_THRESHOLD = 1600; // 安全開關閾值 (Safety Switch Threshold - Ch7)
const int THROTTLE_ARM_PWM = 1080; // 油門解鎖閾值 (Throttle Arming Threshold)
const int WING_STOP_L = 2100;    // 左舵機上揚停止位 (Left Wing Stop Position - UP)
const int WING_STOP_R = 900;     // 右舵機上揚停止位 (Right Wing Stop Position - UP)

// --- 撲翼機動力限制 (Ornithopter Power Limits - PTK 7432 @ 8.4V) ---
const float LIMIT_FREQ_MIN = 1.56;  // 最低拍動頻率 (Minimum Flapping Frequency Hz)
const float LIMIT_FREQ_MAX = 3.125; // 最高拍動頻率 (Maximum Flapping Frequency Hz)
const float US_PER_DEGREE = 11.11;  // 每度對應的微秒換算 (1000us/90deg conversion)
const float AMP_MIN_US = 40.0 * US_PER_DEGREE; // 最低振幅 40度 (Min Amplitude 40deg)
const float AMP_MID_US = 75.0 * US_PER_DEGREE; // 中間振幅 75度 (Mid Amplitude 75deg)
const float AMP_MAX_US = 90.0 * US_PER_DEGREE; // 最高振幅 90度 (Max Amplitude 90deg)
const float THROTTLE_EXPO = 0.5;    // 油門指數曲線因子 (Throttle Expo: 0.0 = linear, 1.0 = cubic)

// --- 全域對象 (Global Objects) ---
DSM2048 rx; // DSMX 接收機對象 (DSMX Receiver Object)
SoftwareSerial debug(PIN_DEBUG_RX, PIN_DEBUG_TX); // 調試串口對象 (Debug Serial Object)

// --- 狀態變數 (State Variables) ---
unsigned long lastMicros = 0;  // 上次主循環微秒時間 (Last loop time in micros)
float cyclePhase = 0.0;        // 拍動循環相位 0~2PI (Flapping Cycle Phase 0~2PI)
bool isSystemArmed = false;    // 系統是否已解鎖 (System Arming State)
bool isInSetupMode = false;    // 是否處於設定模式 (Setup Mode Flag)

int savedOffsetL = 0;          // 左舵機 EEPROM 偏置值 (Saved Left Servo Offset)
int savedOffsetR = 0;          // 右舵機 EEPROM 偏置值 (Saved Right Servo Offset)
bool isGlobalReverse = false;  // 全域正反向標記 (Global Servo Reverse Flag)
int setupStep = 0;             // 當前設定頁面 (Current Setup Step: 0=Trim, 1=Reverse)
unsigned long rudderLeftTimer = 0; // 方向舵左打計時器 (Timer for Rudder-Left-Hold)
bool rudderRightPushed = false; // 方向舵右打防連發 (Flag for Rudder-Right-Click)

// --- 搖桿輸入狀態 (Stick Input States) ---
bool aileronPushed = false;   // 副翼撥動標記 (Aileron stick pushed flag)
bool elevatorPushed = false;  // 升降撥動標記 (Elevator stick pushed flag)

// --- 接收機數據快取 (Receiver Data Cache) ---
volatile int rxThrottle = 1000;
volatile int rxAileron  = 1500;
volatile int rxElevator = 1500;
volatile int rxRudder   = 1500; 
volatile int rxAmpCh    = 1500; 
volatile int rxSafetyCh = 1000; 

// --- PWM 硬體定時器初始化 (Setup Hardware Timer for 333Hz PWM) ---
void setupPWM333Hz() {
  pinMode(PIN_SERVO_L, OUTPUT); 
  pinMode(PIN_SERVO_R, OUTPUT); 
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); 
  ICR1 = 6000; // 設定 3ms 週期 = 333.33Hz (Set 3ms period for 333Hz)
}

// --- 寫入舵機脈寬 (Write PWM Pulse Width in us) ---
void writePWM(int pin, int us) {
  us = constrain(us, SERVO_LIMIT_MIN, SERVO_LIMIT_MAX); // 限制在安全範圍 (Limit to safe range)
  int counts = us * 2; // 將微秒轉換為計時器數值 (Convert us to timer counts)
  if (pin == PIN_SERVO_L) OCR1A = counts;
  else if (pin == PIN_SERVO_R) OCR1B = counts;
}

// --- LED 邏輯處理 (LED Logic Handler) ---
void updateLED() {
  unsigned long now = millis();
  float ampScale = constrain((rxAmpCh - 1000) / 1000.0, 0.0, 1.0);

  if (isInSetupMode) {
    unsigned long cycleTime = 2500; 
    unsigned long t = now % cycleTime;
    bool ledOn = false;

    if (setupStep == 0) { // 1.1 (1L 3S) - Servo Trim Mode
      if (t < 600) ledOn = true;
      else if (t >= 800 && t < 950) ledOn = true;
      else if (t >= 1100 && t < 1250) ledOn = true;
      else if (t >= 1400 && t < 1550) ledOn = true;
    } 
    else if (setupStep == 1) { // 2.1 (2L 2S) - Servo Reverse Mode
      if (t < 600) ledOn = true;
      else if (t >= 800 && t < 1400) ledOn = true;
      else if (t >= 1600 && t < 1750) ledOn = true;
      else if (t >= 1900 && t < 2050) ledOn = true;
    }
    digitalWrite(PIN_LED, ledOn ? HIGH : LOW);
  } 
  else if (!isSystemArmed) {
    digitalWrite(PIN_LED, HIGH);
  } 
  else {
    if (rxThrottle > THROTTLE_ARM_PWM) {
      float flashFreq = 3.0 + ampScale * 12.0; 
      unsigned long flashPeriod = 1000.0 / flashFreq;
      if ((now % flashPeriod) < (flashPeriod / 2)) digitalWrite(PIN_LED, HIGH);
      else digitalWrite(PIN_LED, LOW);
    } else {
      if ((now % 1000) < 500) digitalWrite(PIN_LED, HIGH);
      else digitalWrite(PIN_LED, LOW);
    }
  }
}

void setup() {
  Serial.begin(115200); // USB virtual serial for debugging
  Serial1.begin(115200); 
  debug.begin(38400);   
  debug.println("Butterfly Ornithopter Setup Mode Ready");
  Serial.println("Butterfly Ornithopter Setup Mode Ready");

  EEPROM.get(0, savedOffsetL);
  EEPROM.get(2, savedOffsetR);
  isGlobalReverse = (EEPROM.read(4) == 1);

  if (abs(savedOffsetL) > 450) savedOffsetL = 0;
  if (abs(savedOffsetR) > 450) savedOffsetR = 0;

  setupPWM333Hz(); 
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH); 

  delay(2000); 
  lastMicros = micros(); 
}

void loop() {
  if (rx.gotNewFrame()) {
    uint16_t ch[8]; 
    rx.getChannelValues(ch, 8); 
    rxThrottle  = ch[0]; 
    rxAileron   = ch[1]; 
    rxElevator  = ch[2]; 
    rxRudder    = ch[3]; 
    rxAmpCh     = ch[5]; 
    rxSafetyCh  = ch[6]; 
  }

  // --- USB Serial Debugging (Print when disarmed or in setup mode / 每秒列印一次) ---
  static unsigned long lastDebugPrintTime = 0;
  if (!isSystemArmed && (millis() - lastDebugPrintTime > 1000)) {
    lastDebugPrintTime = millis();
    if (isInSetupMode) {
      Serial.print("[Setup Page "); Serial.print(setupStep == 0 ? "1.1" : "2.1"); Serial.print("] ");
      Serial.print("Thr: "); Serial.print(rxThrottle);
      Serial.print(" | Ail: "); Serial.print(rxAileron);
      Serial.print(" | Ele: "); Serial.print(rxElevator);
      Serial.print(" | Rud: "); Serial.print(rxRudder);
      Serial.print(" | OffsetL: "); Serial.print(savedOffsetL);
      Serial.print(" | OffsetR: "); Serial.println(savedOffsetR);
    } else {
      Serial.print("Thr: "); Serial.print(rxThrottle);
      Serial.print(" | Ail: "); Serial.print(rxAileron);
      Serial.print(" | Ele: "); Serial.print(rxElevator);
      Serial.print(" | Rud: "); Serial.print(rxRudder);
      Serial.print(" | Saf: "); Serial.println(rxSafetyCh);
    }
  }

  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastMicros) / 1000000.0;
  lastMicros = currentMicros;
  if (dt < 0 || dt > 0.1) dt = 0; 

  int outL = WING_STOP_L; 
  int outR = WING_STOP_R; 

  if (rxSafetyCh <= SAFETY_THRESHOLD) {
    isSystemArmed = false; 
    
    if (rxThrottle < 1150 && rxRudder > 1800) {
      if (!isInSetupMode) {
        isInSetupMode = true;
        setupStep = 0;
        rudderRightPushed = true; 
        debug.println("Entering Setup Mode (1.1)");
        Serial.println("Entering Setup Mode (1.1)");
      }
    }

    if (isInSetupMode) {
      // --- Reset Offsets Shortcut (Left stick bottom-right, Right stick top-right for 1.5s) ---
      static unsigned long resetTimer = 0;
      if (rxThrottle < 1150 && rxRudder > 1800 && rxAileron > 1800 && rxElevator > 1800) {
        if (resetTimer == 0) resetTimer = millis();
        if (millis() - resetTimer > 1500) {
          savedOffsetL = 0;
          savedOffsetR = 0;
          debug.println("Offsets Reset to 0.");
          Serial.println("Offsets Reset to 0.");
          // Rapid LED blinking confirmation
          for (int i = 0; i < 5; i++) {
            digitalWrite(PIN_LED, LOW); delay(100);
            digitalWrite(PIN_LED, HIGH); delay(100);
          }
          resetTimer = 0;
        }
      } else {
        resetTimer = 0;
      }

      if (rxRudder > 1800) {
        if (!rudderRightPushed) {
          setupStep = (setupStep + 1) % 2;
          rudderRightPushed = true;
          debug.print("Switch Page: "); debug.println(setupStep == 0 ? "1.1" : "2.1");
          Serial.print("Switch Page: "); Serial.println(setupStep == 0 ? "1.1" : "2.1");
        }
      } else if (rxRudder < 1600) {
        rudderRightPushed = false;
      }

      if (rxRudder < 1200) {
        if (rudderLeftTimer == 0) rudderLeftTimer = millis();
        if (millis() - rudderLeftTimer > 3000) {
          EEPROM.put(0, savedOffsetL);
          EEPROM.put(2, savedOffsetR);
          EEPROM.write(4, isGlobalReverse ? 1 : 0);
          isInSetupMode = false;
          rudderLeftTimer = 0;
          debug.println("Settings Saved.");
          Serial.println("Settings Saved.");
          digitalWrite(PIN_LED, LOW); delay(1000);
          digitalWrite(PIN_LED, HIGH); delay(2000);
          digitalWrite(PIN_LED, LOW); delay(1000);
        }
      } else {
        rudderLeftTimer = 0;
      }

      if (rxRudder > 1400 && rxRudder < 1600) {
        if (setupStep == 0) { // 雙舵微調模式 (Servo Trim Mode)
          // 調整副翼 (Aileron) -> 一邊高一邊低 (Differential trim)
          if (rxAileron > 1900) {
            if (!aileronPushed) {
              savedOffsetL += 11;
              savedOffsetR += 11;
              aileronPushed = true;
            }
          } else if (rxAileron < 1100) {
            if (!aileronPushed) {
              savedOffsetL -= 11;
              savedOffsetR -= 11;
              aileronPushed = true;
            }
          } else {
            aileronPushed = false;
          }

          // 調整升降 (Elevator) -> 兩邊一起高低 (Common-mode trim)
          if (rxElevator > 1900) {
            if (!elevatorPushed) {
              savedOffsetL += 11;
              savedOffsetR -= 11;
              elevatorPushed = true;
            }
          } else if (rxElevator < 1100) {
            if (!elevatorPushed) {
              savedOffsetL -= 11;
              savedOffsetR += 11;
              elevatorPushed = true;
            }
          } else {
            elevatorPushed = false;
          }
        } 
        else if (setupStep == 1) { // 正反向設定模式 (Servo Reverse Mode)
          if (rxAileron > 1900 || rxAileron < 1100) {
            if (!aileronPushed) {
              isGlobalReverse = !isGlobalReverse;
              aileronPushed = true;
              debug.print("Reverse: "); debug.println(isGlobalReverse);
              Serial.print("Reverse: "); Serial.println(isGlobalReverse);
              writePWM(PIN_SERVO_L, PWM_CENTER + savedOffsetL + 200);
              writePWM(PIN_SERVO_R, PWM_CENTER + savedOffsetR + 200);
              delay(150);
            }
          } else aileronPushed = false;
        }
      }
      savedOffsetL = constrain(savedOffsetL, -500, 500);
      savedOffsetR = constrain(savedOffsetR, -500, 500);
      outL = PWM_CENTER + savedOffsetL;
      outR = PWM_CENTER + savedOffsetR;
    } else {
      outL = WING_STOP_L;
      outR = WING_STOP_R;
    }
  } else {
    isInSetupMode = false; 
    if (!isSystemArmed) {
      if (rxThrottle < THROTTLE_ARM_PWM + 20) isSystemArmed = true; 
      else { outL = WING_STOP_L; outR = WING_STOP_R; }
    }
    if (isSystemArmed) {
      int ailInput = (rxAileron - PWM_CENTER);      
      int eleInput = (rxElevator - PWM_CENTER);   
      int rudInput = (rxRudder - PWM_CENTER);   
      int mult = isGlobalReverse ? -1 : 1;
      int commonPitch = (int)(eleInput * 0.44); 
      int diffYaw = (int)(rudInput * 0.44);
      int centerL = PWM_CENTER + savedOffsetL + (commonPitch + diffYaw) * mult; 
      int centerR = PWM_CENTER + savedOffsetR - (commonPitch - diffYaw) * mult; 

      if (rxThrottle > THROTTLE_ARM_PWM) {
        float tScale = constrain((rxThrottle - THROTTLE_ARM_PWM) / (float)(PWM_MAX - THROTTLE_ARM_PWM), 0.0, 1.0);
        // 套用 Expo 指數曲線：tScaleExpo = (1 - k) * tScale + k * tScale^3
        float tScaleExpo = (1.0 - THROTTLE_EXPO) * tScale + THROTTLE_EXPO * tScale * tScale * tScale;
        float currentFreq = LIMIT_FREQ_MIN + (tScaleExpo * (LIMIT_FREQ_MAX - LIMIT_FREQ_MIN));
        float ampScale = constrain((rxAmpCh - 1000) / 1000.0, 0.0, 1.0);
        float currentAmp = AMP_MIN_US + (ampScale * (AMP_MAX_US - AMP_MIN_US));

        // --- 動態舵速安全保護 (Dynamic Servo Speed Protection) ---
        // 確保任何油門(頻率)與振幅組合下，最大角速度不超過負載安全上限 520°/s。
        // 計算公式：A_us_max = (520 * 11.11) / (2 * PI * f) = 919.5 / f
        float maxSafeAmp = 919.5 / currentFreq;
        if (currentAmp > maxSafeAmp) {
          currentAmp = maxSafeAmp;
        }

        cyclePhase += currentFreq * dt * TWO_PI;
        if (cyclePhase > TWO_PI) cyclePhase -= TWO_PI; 
        float diffAmp = ailInput * 0.88; 
        float ampL = currentAmp - diffAmp;
        float ampR = currentAmp + diffAmp;
        if (ampL < 0.0) ampL = 0.0;
        if (ampR < 0.0) ampR = 0.0;
        float wave = sin(cyclePhase);
        outL = centerL + (int)(wave * ampL * mult);
        outR = centerR - (int)(wave * ampR * mult); 
      } else {
        cyclePhase = 0.0;
        outL = centerL;
        outR = centerR;
      }
    }
  }
  writePWM(PIN_SERVO_L, outL);
  writePWM(PIN_SERVO_R, outR);
  updateLED();
}

void serialEvent1() {
  while (Serial1.available()) rx.handleSerialEvent(Serial1.read(), micros());
}