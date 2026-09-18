#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEAdvertisedDevice.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLERemoteService.h>
#include <BLERemoteCharacteristic.h>
#include "TFT_22_ILI9225.h"


#define TFT_RST 26
#define TFT_RS  25
#define TFT_CLK 18
#define TFT_SDI 23
#define TFT_CS   5
#define TFT_LED  4

#define SCAN_TIME_SEC 3       // 每次扫描持续秒数
#define RING_BUF_SIZE 512     // 环形缓冲区大小
#define MIN_SCAN_MS    800    // 最短扫描时间（收集多个广播包以获取最强信号）

// 屏幕坐标映射（以底部圆心为雷达原点）
#define SCREEN_ORIGIN_X  110
#define SCREEN_ORIGIN_Y  173
#define MAX_RANGE_MM     6000  // 雷达最大探测距离 6 米
#define SCALE_PX_PER_MM  (173.0f / MAX_RANGE_MM)  // 约 0.0288 px/mm
#define TARGET_DOT_R     4     // 目标点半径（8px 直径）

// 抗干扰 / 平滑参数
#define EMA_ALPHA        0.35f   // 指数平滑系数 (0~1, 越小越平滑)
#define MAX_JUMP_PX      50      // 帧间最大跳变像素（超过则视为野值丢弃）
#define DRAW_INTERVAL_MS 80      // 屏幕刷新间隔 ms
#define MIN_SPEED_CM_S   10      // 最低运动速度 (cm/s)，低于此值视为静止
#define PERSIST_BOX_MM   400     // 位置锁定判定范围 (mm)：在此范围内的抖动视为鬼影
#define PERSIST_TIMEOUT_MS 4000  // 位置锁定超时 (ms)：困住超过此时间则过滤
#define STATIC_TIMEOUT_MS 3000   // 静止超时：保持 3 秒后才过滤
#define MIN_VALID_Y_MM   320    // Y < 0.32m 属于硬件测量盲区，视为噪声过滤


// HLK-LD2450 蓝牙透传 UUID（16位短格式）
// 实际设备特征: 0xFFF1=Notify(数据上报), 0xFFF2=Write(命令下发)
static BLEUUID svcUUID((uint16_t)0xFFF0);
static BLEUUID writeCharUUID((uint16_t)0xFFF2);
static BLEUUID readCharUUID((uint16_t)0xFFF1);


SPIClass vspi(VSPI);
TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_LED, 255);

BLEScan* pBLEScan = nullptr;
BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pReadChar = nullptr;
BLERemoteCharacteristic* pWriteChar = nullptr;

// 屏幕目标状态
bool anyOnScreen = false;  // 当前是否有目标显示在屏幕上
char gStatusMsg[32] = "";  // 当前状态栏文字

// 记录扫描过程中信号最强的 HLK-LD2450 设备
String bestName = "";
BLEAddress* bestAddr = nullptr;
int bestRSSI = -999;
volatile bool foundTarget = false;  // 扫描回调中置位，触发提前结束扫描
String knownMAC = "";               // 缓存已连接过的 MAC，断连后可直连

// 状态机
enum State { SCANNING, CONNECTING, DISCOVERING, READY };
State state = SCANNING;
unsigned long scanStartTime = 0;
unsigned long lastDataTime = 0;
unsigned long packetCount = 0;
volatile unsigned long notifyCount = 0;  // 回调调用次数（跨任务共享）


// ---- LD2450 雷达数据解析（参照 test/test.cpp） ----

// 帧结构常量
#define FRAME_HEADER_0  0xAA
#define FRAME_HEADER_1  0xFF
#define FRAME_HEADER_2  0x03
#define FRAME_HEADER_3  0x00
#define FRAME_FOOTER_0  0x55
#define FRAME_FOOTER_1  0xCC
#define FRAME_TOTAL_LEN  30
#define TARGET_COUNT      3
#define TARGET_DATA_LEN   8
#define HEADER_LEN        4
#define FOOTER_LEN        2

// 单目标数据结构（8 字节）
struct Target {
  int16_t x;           // X 坐标 (mm)
  int16_t y;           // Y 坐标 (mm)
  int16_t speed;       // 速度 (cm/s)
  uint16_t distanceRes; // 距离分辨率
  bool valid;
};

Target targets[TARGET_COUNT];

// 偏移二进制解码：0x8000 表示 0
int16_t decodeSignedInt16(uint16_t raw) {
  if (raw & 0x8000) {
    return (int16_t)(raw - 0x8000);
  } else {
    return -(int16_t)raw;
  }
}

void parseTarget(const uint8_t* data, Target& t) {
  uint16_t rawX  = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
  uint16_t rawY  = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
  uint16_t rawSp = (uint16_t)data[4] | ((uint16_t)data[5] << 8);

  t.x           = decodeSignedInt16(rawX);
  t.y           = decodeSignedInt16(rawY);
  t.speed       = decodeSignedInt16(rawSp);
  t.distanceRes = (uint16_t)data[6] | ((uint16_t)data[7] << 8);
  t.valid       = (rawX != 0 || rawY != 0 || rawSp != 0)
                  && (t.y >= MIN_VALID_Y_MM);  // 过滤盲区噪声
}

void printTargets() {
  Serial.println("--- Radar Targets ---");
  for (int i = 0; i < TARGET_COUNT; i++) {
    if (targets[i].valid) {
      Serial.printf("  T%d: X=%+.2fm  Y=%+.2fm  Speed=%+.2fm/s  DistRes=%umm\n",
                    i + 1,
                    targets[i].x / 1000.0f,
                    targets[i].y / 1000.0f,
                    targets[i].speed / 100.0f,
                    targets[i].distanceRes);
    } else {
      Serial.printf("  T%d: --\n", i + 1);
    }
  }
}

// 平滑后的目标屏幕坐标（float 精度，避免累积误差）
float   smoothSX[TARGET_COUNT];
float   smoothSY[TARGET_COUNT];
bool    smoothInit[TARGET_COUNT];   // 是否已有历史值

// 记录上一帧已绘制的屏幕坐标，用于擦除
int16_t prevScreenX[TARGET_COUNT];
int16_t prevScreenY[TARGET_COUNT];
bool    prevValid[TARGET_COUNT];
unsigned long staticSinceMs[TARGET_COUNT];  // 进入静止状态的起始时间

// 鬼影检测：追踪目标在雷达坐标系中的活动范围边界
int16_t ghostMinX[TARGET_COUNT], ghostMaxX[TARGET_COUNT];
int16_t ghostMinY[TARGET_COUNT], ghostMaxY[TARGET_COUNT];
unsigned long ghostStartMs[TARGET_COUNT];
bool ghostMarked[TARGET_COUNT];

// 轨迹历史：每个目标保存最近 5 个不同的屏幕位置
#define TRAIL_LEN 5
int16_t trailX[TARGET_COUNT][TRAIL_LEN];
int16_t trailY[TARGET_COUNT][TRAIL_LEN];
uint8_t trailCount[TARGET_COUNT];  // 当前有效轨迹点数 (0..TRAIL_LEN)

// 拖影渐变色：5 级绿色（最旧→最新），用于轨迹点绘制
const uint16_t TRAIL_COLORS[TRAIL_LEN] = {
  0x0120,  // 最暗 (G≈9)
  0x0240,  // 
  0x0360,  //
  0x04A0,  //
  0x0600,  // 最亮 (G≈48)，但仍暗于主目标点 COLOR_GREEN
};

// 初始化/重置单个目标的鬼影检测状态
void ghostReset(int i) {
  ghostStartMs[i] = 0;
  ghostMarked[i] = false;
}

// 将雷达坐标映射到屏幕坐标
void radarToScreen(int16_t rx_mm, int16_t ry_mm, int16_t& sx, int16_t& sy) {
  sx = SCREEN_ORIGIN_X + (int16_t)(rx_mm * SCALE_PX_PER_MM);
  sy = SCREEN_ORIGIN_Y - (int16_t)(ry_mm * SCALE_PX_PER_MM);
}

// 绘制圆弧（逐段直线逼近）
void drawArc(int16_t cx, int16_t cy, uint16_t r,
             float startDeg, float endDeg, uint16_t color) {
  const float step = 2.0f;  // 每段 2°
  for (float deg = startDeg; deg < endDeg; deg += step) {
    float nextDeg = deg + step;
    if (nextDeg > endDeg) nextDeg = endDeg;
    float a1 = deg * PI / 180.0f;
    float a2 = nextDeg * PI / 180.0f;
    int16_t x1 = cx + (int16_t)(r * cosf(a1));
    int16_t y1 = cy + (int16_t)(r * sinf(a1));
    int16_t x2 = cx + (int16_t)(r * cosf(a2));
    int16_t y2 = cy + (int16_t)(r * sinf(a2));
    tft.drawLine(x1, y1, x2, y2, color);
  }
}

// 绘制背景图层：原点 + 两条视场角线 + 两条圆弧（叠加，不清屏）
void drawBackground() {
  tft.fillCircle(110, 173, 2, COLOR_GREEN);
  tft.drawLine(110, 173,   0, 110, COLOR_GREEN);  // 左线
  tft.drawLine(110, 173, 220, 110, COLOR_GREEN);  // 右线

  // 圆弧1：半径 80px，起止于左右两条斜线
  float a1_start = atan2f(110.0f - 173,   0.0f - 110) * 180.0f / PI;
  float a1_end   = atan2f(110.0f - 173, 220.0f - 110) * 180.0f / PI;
  drawArc(110, 173,  40, a1_start, a1_end, COLOR_DARKGREEN);
  drawArc(110, 173,  80, a1_start, a1_end, COLOR_DARKGREEN);
  drawArc(110, 173, 120, a1_start, a1_end, COLOR_DARKGREEN);

  // 圆弧3：半径 160px，起止于屏幕左右边缘 (X=0, X=220)
  int16_t yEdge = 173 - (int16_t)sqrtf(160.0f * 160.0f - 110.0f * 110.0f);
  float a2_start = atan2f(yEdge - 173.0f,   0.0f - 110) * 180.0f / PI;
  float a2_end   = atan2f(yEdge - 173.0f, 220.0f - 110) * 180.0f / PI;
  drawArc(110, 173, 160, a2_start, a2_end, COLOR_DARKGREEN);
}

// 告警闪烁：暗红色矩形边框闪烁 2 次（3px 宽，覆盖全屏 220×176）
void showAlert() {
  for (int flash = 0; flash < 2; flash++) {
    for (int i = 0; i < 3; i++) {
      tft.drawRectangle(i, i, 219 - i, 175 - i, COLOR_DARKRED);
    }
    delay(100);
    for (int i = 0; i < 3; i++) {
      tft.drawRectangle(i, i, 219 - i, 175 - i, COLOR_BLACK);
    }
    delay(100);
  }
  drawBackground();
}

// 屏幕顶部居中显示状态文字（白色，Terminal6x8 字体）
void showStatus(const char* msg) {
  strncpy(gStatusMsg, msg, sizeof(gStatusMsg) - 1);
  gStatusMsg[sizeof(gStatusMsg) - 1] = '\0';

  tft.fillRectangle(0, 0, 219, 8, COLOR_BLACK);  // 清除状态行

  int len = strlen(msg);
  int x = (220 - len * 6) / 2;  // Terminal6x8: 6px/字符，居中
  if (x < 0) x = 0;
  tft.drawText(x, 0, msg, COLOR_WHITE);
}

// 向目标 i 的轨迹数组追加新位置（仅当与末尾位置不同时）
void pushTrail(int i, int16_t x, int16_t y) {
  if (trailCount[i] > 0 && trailX[i][trailCount[i] - 1] == x
                          && trailY[i][trailCount[i] - 1] == y) {
    return;  // 位置未变，不重复记录
  }
  if (trailCount[i] < TRAIL_LEN) {
    trailX[i][trailCount[i]] = x;
    trailY[i][trailCount[i]] = y;
    trailCount[i]++;
  } else {
    // 数组已满，整体左移，新值放入末尾
    for (int j = 0; j < TRAIL_LEN - 1; j++) {
      trailX[i][j] = trailX[i][j + 1];
      trailY[i][j] = trailY[i][j + 1];
    }
    trailX[i][TRAIL_LEN - 1] = x;
    trailY[i][TRAIL_LEN - 1] = y;
  }
}

// 擦除旧点 + 绘制新点（含 EMA 平滑 + 野值过滤 + 帧率限制）
void drawTargets() {
  static unsigned long lastDraw = 0;
  unsigned long now = millis();
  if (now - lastDraw < DRAW_INTERVAL_MS) return;
  lastDraw = now;

  for (int i = 0; i < TARGET_COUNT; i++) {
    if (targets[i].valid) {
      // 静止杂波过滤（3 秒保持：短暂静止的人不消失，静止超时才过滤）
      if (abs(targets[i].speed) < MIN_SPEED_CM_S) {
        if (staticSinceMs[i] == 0) {
          staticSinceMs[i] = now;                 // 开始计时
        } else if (now - staticSinceMs[i] >= STATIC_TIMEOUT_MS) {
          // 超时 → 视为固定物体，擦除
          if (prevValid[i]) {
            tft.fillCircle(prevScreenX[i], prevScreenY[i], TARGET_DOT_R, COLOR_BLACK);
            prevValid[i] = false;
            smoothInit[i] = false;
          }
          continue;
        }
        // 未超时，继续显示（走后续绘制流程）
      } else {
        staticSinceMs[i] = 0;                      // 恢复运动，重置计时
      }

      // 鬼影过滤（多径反射假目标：速度在闪烁但位置锁死）
      if (!ghostMarked[i]) {
        if (ghostStartMs[i] != 0) {
          int16_t drx = abs(targets[i].x - ghostMinX[i]);
          int16_t dry = abs(targets[i].y - ghostMinY[i]);
          if (drx <= PERSIST_BOX_MM && dry <= PERSIST_BOX_MM) {
            if (now - ghostStartMs[i] >= PERSIST_TIMEOUT_MS) {
              ghostMarked[i] = true;
            }
          } else {
            // 脱离锁定区 → 重置
            ghostMinX[i] = targets[i].x;
            ghostMinY[i] = targets[i].y;
            ghostStartMs[i] = now;
          }
        } else {
          ghostMinX[i] = targets[i].x;
          ghostMinY[i] = targets[i].y;
          ghostStartMs[i] = now;
        }
      }
      if (ghostMarked[i]) {
        // 检查是否脱离鬼影区域
        if (abs(targets[i].x - ghostMinX[i]) > PERSIST_BOX_MM ||
            abs(targets[i].y - ghostMinY[i]) > PERSIST_BOX_MM) {
          ghostReset(i);
        } else {
          if (prevValid[i]) {
            tft.fillCircle(prevScreenX[i], prevScreenY[i], TARGET_DOT_R, COLOR_BLACK);
            prevValid[i] = false;
            smoothInit[i] = false;
          }
          staticSinceMs[i] = 0;
          continue;
        }
      }

      int16_t rawSX, rawSY;
      radarToScreen(targets[i].x, targets[i].y, rawSX, rawSY);

      // 超出屏幕边界 → 视为无效
      if (rawSX < 0 || rawSX > 220 || rawSY < 0 || rawSY > 176) {
        continue;
      }

      // 野值过滤：帧间跳变超过阈值则丢弃本次更新
      if (smoothInit[i]) {
        float dx = rawSX - smoothSX[i];
        float dy = rawSY - smoothSY[i];
        float dist = sqrtf(dx * dx + dy * dy);
        if (dist > MAX_JUMP_PX) {
          continue;  // 野值，跳过
        }
      }

      // EMA 指数平滑
      if (!smoothInit[i]) {
        smoothSX[i] = (float)rawSX;
        smoothSY[i] = (float)rawSY;
        smoothInit[i] = true;
      } else {
        smoothSX[i] = EMA_ALPHA * rawSX + (1.0f - EMA_ALPHA) * smoothSX[i];
        smoothSY[i] = EMA_ALPHA * rawSY + (1.0f - EMA_ALPHA) * smoothSY[i];
      }

      int16_t sx = (int16_t)(smoothSX[i] + 0.5f);
      int16_t sy = (int16_t)(smoothSY[i] + 0.5f);

      // 平滑后再次检查边界
      if (sx < 0 || sx > 220 || sy < 0 || sy > 176) {
        continue;
      }

      // 坐标未变化则跳过绘制（减少 SPI 流量）
      if (prevValid[i] && sx == prevScreenX[i] && sy == prevScreenY[i]) {
        continue;
      }

      // 快照旧轨迹，用于擦除
      int16_t oldTrailX[TRAIL_LEN], oldTrailY[TRAIL_LEN];
      uint8_t oldCount = trailCount[i];
      memcpy(oldTrailX, trailX[i], sizeof(oldTrailX));
      memcpy(oldTrailY, trailY[i], sizeof(oldTrailY));

      // 擦除旧点（上一帧主目标点）
      if (prevValid[i]) {
        tft.fillCircle(prevScreenX[i], prevScreenY[i], TARGET_DOT_R, COLOR_BLACK);
      }

      // 绘制新点（主目标点）
      tft.fillCircle(sx, sy, TARGET_DOT_R, COLOR_GREEN);
      pushTrail(i, sx, sy);  // 记录轨迹

      // 擦除旧轨迹点
      for (int t = 0; t < oldCount; t++) {
        if (oldTrailX[t] >= 0 && oldTrailX[t] < 220 && oldTrailY[t] >= 0 && oldTrailY[t] < 176) {
          tft.fillCircle(oldTrailX[t], oldTrailY[t], 2, COLOR_BLACK);
        }
      }
      // 绘制新轨迹点（渐暗拖影）
      for (int t = 0; t < trailCount[i]; t++) {
        if (trailX[i][t] >= 0 && trailX[i][t] < 220 && trailY[i][t] >= 0 && trailY[i][t] < 176) {
          int ci = TRAIL_LEN - trailCount[i] + t;  // 0=最暗 → 4=最亮
          tft.fillCircle(trailX[i][t], trailY[i][t], 2, TRAIL_COLORS[ci]);
        }
      }

      prevScreenX[i] = sx;
      prevScreenY[i] = sy;
      prevValid[i] = true;
    } else {
      // 目标消失 → 擦除旧点 + 重置所有状态
      if (prevValid[i]) {
        tft.fillCircle(prevScreenX[i], prevScreenY[i], TARGET_DOT_R, COLOR_BLACK);
        // 擦除轨迹点
        for (int t = 0; t < trailCount[i]; t++) {
          if (trailX[i][t] >= 0 && trailX[i][t] < 220 && trailY[i][t] >= 0 && trailY[i][t] < 176) {
            tft.fillCircle(trailX[i][t], trailY[i][t], 2, COLOR_BLACK);
          }
        }
        trailCount[i] = 0;
        prevValid[i] = false;
        smoothInit[i] = false;
      }
      staticSinceMs[i] = 0;
      ghostReset(i);
    }
  }

  // 更新屏幕目标状态
  anyOnScreen = false;
  for (int i = 0; i < TARGET_COUNT; i++) {
    if (prevValid[i]) { anyOnScreen = true; break; }
  }
}

// 环形缓冲（bufHead 由 BLE 回调写入，需 volatile）
uint8_t ringBuf[RING_BUF_SIZE];
volatile uint16_t bufHead = 0;
uint16_t bufTail = 0;
uint8_t frameBuf[FRAME_TOTAL_LEN];
uint8_t frameIdx = 0;

void ringPush(uint8_t b) {
  ringBuf[bufHead] = b;
  bufHead = (bufHead + 1) % RING_BUF_SIZE;
}

uint16_t ringAvail() {
  return (bufHead - bufTail + RING_BUF_SIZE) % RING_BUF_SIZE;
}

// 从环形缓冲取一个字节
int ringPop() {
  if (ringAvail() == 0) return -1;
  uint8_t b = ringBuf[bufTail];
  bufTail = (bufTail + 1) % RING_BUF_SIZE;
  return b;
}

// 尝试解析一帧（参照 test.cpp 的帧尾检测 + 帧头扫描算法）
bool tryParseFrame() {
  // 从环形缓冲逐个取字节，累积到 frameBuf
  while (ringAvail() > 0) {
    int b = ringPop();
    if (b < 0) break;

    if (frameIdx < sizeof(frameBuf)) {
      frameBuf[frameIdx++] = (uint8_t)b;
    }

    // 检测帧尾 55 CC（最后两字节）
    if (frameIdx >= 2 &&
        frameBuf[frameIdx - 2] == FRAME_FOOTER_0 &&
        frameBuf[frameIdx - 1] == FRAME_FOOTER_1) {

      // 在累积的数据中搜索完整帧头 AA FF 03 00
      int headerPos = -1;
      for (int i = 0; i <= (int)frameIdx - FRAME_TOTAL_LEN; i++) {
        if (frameBuf[i]     == FRAME_HEADER_0 &&
            frameBuf[i + 1] == FRAME_HEADER_1 &&
            frameBuf[i + 2] == FRAME_HEADER_2 &&
            frameBuf[i + 3] == FRAME_HEADER_3) {
          // 确认帧尾在正确位置
          if (frameBuf[i + 28] == FRAME_FOOTER_0 &&
              frameBuf[i + 29] == FRAME_FOOTER_1) {
            headerPos = i;
            break;
          }
        }
      }

      if (headerPos >= 0) {
        // 解析帧数据
        const uint8_t* frame = frameBuf + headerPos;
        for (int i = 0; i < TARGET_COUNT; i++) {
          parseTarget(frame + HEADER_LEN + i * TARGET_DATA_LEN, targets[i]);
        }
        packetCount++;
        printTargets();
        drawTargets();
        frameIdx = 0;
        return true;
      } else {
        // 帧尾匹配但帧头不正确，丢弃最旧字节重新搜索
        if (frameIdx > 0) {
          memmove(frameBuf, frameBuf + 1, frameIdx - 1);
          frameIdx--;
        }
      }
    }

    // 防止 frameBuf 溢出
    if (frameIdx >= sizeof(frameBuf)) {
      memmove(frameBuf, frameBuf + 1, sizeof(frameBuf) - 1);
      frameIdx = sizeof(frameBuf) - 1;
    }
  }
  return false;
}


// BLE 通知回调（在 BLE 栈任务中执行，仅做轻量操作避免栈溢出）
static void onNotify(BLERemoteCharacteristic* pChar,
                     uint8_t* pData, size_t length,
                     bool isNotify) {
  lastDataTime = millis();
  notifyCount++;
  for (size_t i = 0; i < length; i++) {
    ringPush(pData[i]);
  }
}


// BLE 扫描回调：发现设备时触发
class ScanCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    String name = advertisedDevice.getName().c_str();
    if (name.startsWith("HLK-LD2450")) {
      int rssi = advertisedDevice.getRSSI();
      Serial.print("[发现] ");
      Serial.print(name);
      Serial.print(" | MAC: ");
      Serial.print(advertisedDevice.getAddress().toString().c_str());
      Serial.print(" | RSSI: ");
      Serial.print(rssi);
      Serial.println(" dBm");

      // 更新最强信号设备
      if (rssi > bestRSSI) {
        bestRSSI = rssi;
        bestName = name;
        if (bestAddr != nullptr) {
          delete bestAddr;
        }
        bestAddr = new BLEAddress(advertisedDevice.getAddress());
      }
      foundTarget = true;  // 发现目标设备，通知主循环可提前结束扫描
    }
  }
};


// 重置状态，准备重新扫描
void resetAndRescan() {
  pReadChar = nullptr;
  pWriteChar = nullptr;
  if (pClient != nullptr) {
    pClient->disconnect();
    pClient = nullptr;
  }
  if (bestAddr != nullptr) {
    delete bestAddr;
    bestAddr = nullptr;
  }
  bestRSSI = -999;
  foundTarget = false;
  bufHead = bufTail = 0;
  frameIdx = 0;

  // 如果已知 MAC，直接尝试连接（跳过扫描）
  if (knownMAC.length() > 0) {
    Serial.printf("直连已知设备: %s\n", knownMAC.c_str());
    bestAddr = new BLEAddress(knownMAC.c_str());
    bestName = "HLK-LD2450（直连）";
    pClient = BLEDevice::createClient();
    showStatus("Connecting...");
    state = CONNECTING;
    return;
  }

  delay(500);  // 短延时让 BLE 栈复位
  Serial.println("开始重新扫描...\n");
  showStatus("Scanning...");
  pBLEScan->start(SCAN_TIME_SEC, false);
  scanStartTime = millis();
  state = SCANNING;
}


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("===== BLE: 连接 HLK-LD2450 毫米波雷达 =====");

  // 初始化 TFT 显示屏
  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, HIGH);
  vspi.begin(TFT_CLK, -1, TFT_SDI, TFT_CS);
  tft.begin(vspi);
  tft.setOrientation(3);
  tft.setBackgroundColor(COLOR_BLACK);
  tft.setFont(Terminal6x8);
  drawBackground();
  showStatus("Scanning...");

  // 初始化 BLE 扫描
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new ScanCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  // 开始第一轮扫描
  Serial.println("开始扫描 BLE 设备...\n");
  pBLEScan->start(SCAN_TIME_SEC, false);
  scanStartTime = millis();
  state = SCANNING;
}


void loop() {
  switch (state) {
    case SCANNING: {
      unsigned long elapsed = millis() - scanStartTime;
      // 发现目标设备后，等够最短扫描时间即提前结束（收集更优信号强度）
      if (foundTarget && elapsed >= MIN_SCAN_MS) {
        elapsed = SCAN_TIME_SEC * 1000;  // 触发提前结束
      }
      if (elapsed >= SCAN_TIME_SEC * 1000) {
        pBLEScan->clearResults();
        Serial.printf("---------- 扫描结束 (耗时 %lums) ----------\n", elapsed);

        if (bestAddr != nullptr) {
          Serial.print("最强信号设备: ");
          Serial.print(bestName);
          Serial.print(" | MAC: ");
          Serial.print(bestAddr->toString().c_str());
          Serial.print(" | RSSI: ");
          Serial.print(bestRSSI);
          Serial.println(" dBm");
          Serial.println("正在连接...\n");

          pClient = BLEDevice::createClient();
          showStatus("Connecting...");
          state = CONNECTING;
        } else {
          Serial.println("未发现 HLK-LD2450 设备\n");
          resetAndRescan();
        }
      }
      break;
    }

    case CONNECTING: {
      if (pClient->connect(*bestAddr)) {
        Serial.print("连接成功！设备: ");
        Serial.println(bestName);
        Serial.print("MAC: ");
        Serial.println(bestAddr->toString().c_str());

        // 缓存 MAC，断连后可直连跳过扫描
        knownMAC = bestAddr->toString().c_str();

        // 协商 MTU，提高数据传输效率
        delay(200);
        uint16_t mtu = pClient->getMTU();
        Serial.printf("当前 MTU: %u\n", mtu);
        pClient->setMTU(517);
        delay(200);

        Serial.println("正在发现服务...");
        showStatus("Discovering services...");
        state = DISCOVERING;
      } else {
        Serial.println("连接失败，即将重新扫描...\n");
        knownMAC = "";  // 清除缓存避免循环失败
        resetAndRescan();
      }
      break;
    }

    case DISCOVERING: {
      BLERemoteService* pService = pClient->getService(svcUUID);
      if (pService == nullptr) {
        Serial.print("未找到透传服务 0xFFF0\n");
        Serial.println("尝试列出所有服务...");
        std::map<std::string, BLERemoteService*>* svcMap = pClient->getServices();
        if (svcMap != nullptr) {
          for (auto& kv : *svcMap) {
            Serial.printf("  服务 UUID: %s\n", kv.second->getUUID().toString().c_str());
          }
        }
        knownMAC = "";  // 清除缓存，下次走完整扫描
        resetAndRescan();
        break;
      }
      Serial.println("已找到透传服务 (0xFFF0)");

      delay(300);  // 等待特征发现完成

      // 获取 Read/Notify 特征（先用 UUID 查，失败则枚举回退）
      pReadChar = pService->getCharacteristic(readCharUUID);
      if (pReadChar == nullptr) {
        Serial.println("UUID 查找 Read 特征失败，枚举所有特征...");
        std::map<std::string, BLERemoteCharacteristic*>* charMap =
            pService->getCharacteristics();
        if (charMap != nullptr) {
          for (auto& kv : *charMap) {
            Serial.printf("  特征 UUID: %s", kv.second->getUUID().toString().c_str());
            if (kv.second->canNotify()) Serial.print(" [Notify]");
            if (kv.second->canIndicate()) Serial.print(" [Indicate]");
            if (kv.second->canWrite()) Serial.print(" [Write]");
            if (kv.second->canRead()) Serial.print(" [Read]");
            Serial.println();
            // 匹配 0xFFF1 (Read/Notify)
            if (kv.second->getUUID().equals(readCharUUID)) {
              pReadChar = kv.second;
            }
            // 匹配 0xFFF2 (Write)
            if (kv.second->getUUID().equals(writeCharUUID)) {
              pWriteChar = kv.second;
            }
          }
        }
      } else {
        Serial.println("已找到 Read/Notify 特征 (0xFFF1)");
      }

      if (pReadChar == nullptr) {
        Serial.println("错误: 未找到 0xFFF1 特征\n");
        knownMAC = "";  // 清除缓存
        resetAndRescan();
        break;
      }

      // 获取 Write 特征
      if (pWriteChar == nullptr) {
        pWriteChar = pService->getCharacteristic(writeCharUUID);
      }
      if (pWriteChar != nullptr) {
        Serial.println("已找到 Write 特征 (0xFFF2)");
      }

      // 注册通知回调
      if (pReadChar->canNotify()) {
        pReadChar->registerForNotify(onNotify, true);
        Serial.println("已注册 Notify 回调\n");
      } else if (pReadChar->canIndicate()) {
        pReadChar->registerForNotify(onNotify, false);
        Serial.println("已注册 Indicate 回调\n");
      } else {
        Serial.println("Read 特征不支持通知/指示\n");
        knownMAC = "";  // 清除缓存
        resetAndRescan();
        break;
      }

      bufHead = bufTail = 0;
      Serial.println("========== 雷达数据接收中 ==========\n");
      lastDataTime = millis();
      packetCount = 0;
      showStatus(("Connected: " + bestName).c_str());
      state = READY;
      break;
    }

    case READY: {
      if (!pClient->isConnected()) {
        Serial.println("\n连接已断开，即将重新扫描...");
        resetAndRescan();
        break;
      }

      // 在主循环中解析帧（避免在 BLE 回调栈中做重操作导致栈溢出）
      while (tryParseFrame()) { /* 持续解析直到缓冲区空 */ }

      // 目标从无到有首次出现 → 触发告警闪烁
      static bool wasOnScreen = false;
      if (anyOnScreen && !wasOnScreen) {
        showAlert();
        showStatus(gStatusMsg);  // 告警后恢复状态栏
      }
      wasOnScreen = anyOnScreen;

      // 每 2 秒输出诊断信息
      static unsigned long lastDiag = 0;
      if (millis() - lastDiag >= 2000) {
        lastDiag = millis();
        uint16_t avail = ringAvail();
        Serial.printf("[诊断] 通知回调:%lu次 | 缓冲待处理:%uB | 已解析:%lu帧\n",
                      notifyCount, avail, packetCount);
      }

      // 每 1 秒重绘背景（覆盖目标点可能造成的线条残缺）
      static unsigned long lastBgRedraw = 0;
      if (millis() - lastBgRedraw >= 1000) {
        lastBgRedraw = millis();
        drawBackground();
      }

      if (millis() - lastDataTime > 30000 && lastDataTime > 0) {
        Serial.println("30 秒未收到数据，连接仍保持中...");
        lastDataTime = millis();
      }

      delay(10);
      break;
    }
  }

  delay(10);
}