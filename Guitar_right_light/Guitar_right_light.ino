// ============================================================================
//  Wearable Guitar - 오른손 스트럼 컨트롤러 (DMP 자세/스윙 게이팅 버전)
// ----------------------------------------------------------------------------
//  교수님의 mpu6500_dmp6_rabbit DMP 코드를 ESP32(Lotin Lite)로 이식.
//  - 쿼터니언/중력벡터로 "손목 자세(연주 자세인지)"를 판별 -> 자세 게이트
//  - 중력 제거 선형가속도의 "위아래(중력축) 성분"이 우세할 때만 스트럼 인정
//    -> 아무 방향 흔들림 / 수평 흔들림 / 비연주 올리기·내리기 동작을 차단
//
//  배선: SDA -> GPIO19, SCL -> GPIO22 (INT핀 불필요, FIFO 폴링 방식)
//  부팅 직후 "연주 자세로 디바이스를 들고 약 2초간 정지" -> 기준 자세 캘리브레이션
// ============================================================================

#include "I2Cdev.h"
#include "MPU6500_9Axis_MotionApps41.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

// ---------------------------------------------------------------------------
//  디버그 모드
//  - 정의하면 STRUM:vel 트리거 외에 사람이 읽을 수 있는 센서/게이트 값도 출력한다.
//  - 임계값 튜닝용. 평소(브릿지 연결)에는 주석 처리해 STRUM:vel 만 깔끔히 내보낸다.
//    (디버그 라인을 켜도 midi.html 파서는 "STRUM:"으로 시작하는 줄만 처리한다.)
// ---------------------------------------------------------------------------
#define DEBUG_TUNE

// ---------------------------------------------------------------------------
//  DMP 사용 여부
//  - 이 MPU6500 모듈은 우노에서는 DMP가 정상 동작하지만(칩 정상 확인됨),
//    ESP32에서는 DMP 펌웨어(~2KB) 전송 중 I2C가 크래시/버스행에 빠져 실패한다.
//    실패한 DMP 시도는 버스를 먹통으로 만들어 폴백까지 망가뜨리므로, ESP32에서는
//    아예 DMP를 건너뛰고 안정적인 폴백 경로로 곧장 부팅한다.
//  - 우노 등 DMP가 동작하는 보드에서 쓰려면 1로 바꾼다.
// ---------------------------------------------------------------------------
#define USE_DMP 0

// ---------------------------------------------------------------------------
//  하드웨어/센서 설정
// ---------------------------------------------------------------------------
#define I2C_SDA 19
#define I2C_SCL 22

// 손목 안/밖(연주 자세)을 판별할 자세 축. ypr 인덱스: 2=roll, 1=pitch
// 실측 결과 이 장착에서는 손목 안/밖이 pitch로 나타나므로 1(pitch) 사용.
#define ORIENT_AXIS 1

// 자세 게이트 사용 여부. 1이면 연주 자세(기준 대비 자세각 편차 < POSE_TOL_DEG)
// 일 때만 스트럼 허용. 0이면 자세 무관(브링업/진단용).
#define USE_POSE_GATE 0

// ---------------------------------------------------------------------------
//  게이팅 임계값 (현장 튜닝 대상)
// ---------------------------------------------------------------------------
// 자세 게이트: 기준(연주) 자세에서 허용하는 자세각(ORIENT_AXIS) 편차(도).
// |dPitch|(또는 dRoll)가 이 값보다 크면 연주 자세가 아니라고 보고 차단한다.
// 안쪽(치기)~바깥쪽(허공) 차이가 약 9°라 그 사이값으로 튜닝. 라이브 로그의
// dPitch를 보며 "치기 자세에선 안 넘고, 허공/올리기 자세에선 넘는" 값으로 조정.
const float POSE_TOL_DEG = 8.0f;

// 스윙 게이트(중력 = 1g 기준의 배수). 캘리브레이션으로 측정한 gravLSB에 곱해 사용.
// STRUM_G를 높일수록 "큰 스윙"에만 반응(작은 흔들림 무시). 작게 낮추면 민감해진다.
const float STRUM_G   = 0.50f; // 이 값(×1g) 이상의 수직 선형가속도에서 스트럼 시작
const float RELEASE_G = 0.20f; // 이 값 아래로 떨어져야 다음 스트럼 재무장
const float ACCEL_MAX_G = 2.5f; // 벨로시티 최대치로 매핑할 수직가속 상한

// "위아래" 판정: 수평 성분이 수직 성분의 이 비율보다 작아야 스트럼으로 인정
const float HORIZ_DOM_RATIO = 0.8f;

const int PEAK_WINDOW_MS = 25; // 타격 최고점 추적 구간

// 불응기(refractory): 한 번 친 뒤 이 시간 동안은 새 스트럼을 무시한다.
// 한 번의 큰 스윙 안의 "가속->감속->반동"이 여러 발로 쪼개지는 걸 막는 핵심값.
// 너무 크면 빠른 연속 스트럼을 놓치고, 너무 작으면 한 동작이 여러 번 잡힌다.
const unsigned long STRUM_REFRACTORY_MS = 150;

// 재무장 조건 (둘 중 하나면 다음 스트럼 허용):
//  (a) 연속 스트럼: 스트럼축 자이로가 직전 발사와 "반대 부호"로 STRUM_REVERSE_GYRO 이상
//      돌면 = 반대 스트로크 시작 -> 즉시 재무장. (위아래 쉬지 않고 칠 때 핵심)
//  (b) 단발 스트럼: vert가 RELEASE 아래로 STRUM_QUIET_MS 이상 연속 머물면 = 동작 종료.
// 같은 스트로크 안의 가속→감속 반동은 부호가 안 바뀌므로 (a)에 안 걸려 중복 발사를 막는다.
const unsigned long STRUM_QUIET_MS = 130;
const int STRUM_REVERSE_GYRO = 8000; // LSB. 반대 스트로크로 인정할 자이로 최소 크기

// 업/다운 방향 부호. 장착 방향 탓에 내려치기가 U로, 올려치기가 D로 잡히면 1로 바꿔 뒤집는다.
// (실측 로그: 다운스트로크가 Gz+ 인데 U로 찍혀 뒤집힘 -> 0으로 바로잡음)
#define INVERT_STRUM_DIR 0

// 방향 판정 방식:
//  - 스트럼은 손목 "회전"이라 스윙 내내 자이로(각속도) 부호가 일정하다(가속도는
//    스윙 도중 +/-가 뒤집혀 방향 판정이 불안정). 그래서 자이로 부호를 "주 신호"로 쓴다.
//  - STRUM_GYRO_AXIS: 방향을 읽을 자이로축. -1=자동(스윙 피크에서 |값|이 가장 큰 축),
//    0=X, 1=Y, 2=Z. 실측 로그상 이 마운트에선 스윙이 Z축 회전으로 일관되게 잡혀 Z(2)로 고정.
//    (마운트를 바꾸면 -1로 두고 DEBUG_TUNE에서 어느 축이 일관된지 확인 후 다시 고정)
#define STRUM_GYRO_AXIS 2

// --- A(y) 보조 융합 -----------------------------------------------------------
// 실측 로그상 이 마운트에선 자이로 Z와 A(y) 선형성분이 "완벽한 반(反)상관"이다
//   (Gz+ ↔ Ay-,  Gz- ↔ Ay+).  즉 같은 방향 정보를 반대 부호로 담고 있다.
// 그래서 방향을 자이로 단독이 아니라 (정규화된 자이로) + (정규화된 -A(y)) 의
// 가중 합 부호로 정한다. 자이로가 포화·노이즈로 애매할 때 A(y)가 받쳐준다.
//  - DIR_USE_ACCEL: 0이면 A(y) 무시(자이로 단독), 1이면 융합.
//  - DIR_ACC_SIGN : Ay와 다운방향의 상관 부호. 이 마운트는 반상관이라 -1.
//    (마운트를 바꿔 Gz+ ↔ Ay+ 로 같은부호가 되면 +1로 바꾼다)
//  - *_REF : 정규화 기준 크기(LSB). 실측 피크(|Gz|≈10k~32k, |Ay|≈1k~6k)를 반영.
//  - W_*   : 가중치. 자이로가 더 신뢰되므로 자이로를 크게 둔다.
#define DIR_USE_ACCEL 1
#define DIR_ACC_SIGN  (-1)
const float DIR_GYRO_REF = 12000.0f;
const float DIR_ACC_REF  = 3000.0f;
const float DIR_W_GYRO   = 1.0f;
const float DIR_W_ACC    = 0.6f;

// ---------------------------------------------------------------------------
//  폴백(레지스터 직접 읽기) 모드
//  - MPU-6500은 6축이라 9축 MotionApps DMP가 켜지지 않으므로 보통 이 경로를 사용.
//  - DMP 경로와 동일한 알고리즘: 가속도에서 저역통과로 중력벡터를 추정하고,
//    (1) 중력 기준 손목 roll 자세 게이트, (2) 중력축 수직가속 vs 수평가속 분리.
//  - 임계값/허용범위 상수(STRUM_G, RELEASE_G, ACCEL_MAX_G, POSE_TOL_DEG,
//    HORIZ_DOM_RATIO)는 DMP 경로와 공유한다.
// ---------------------------------------------------------------------------
// 중력 추정 저역통과 시간상수(초). 자세 게이트를 쓰지 않으므로(USE_POSE_GATE=0)
// 크게 둘 이유가 없다. 작게 잡아야 자세를 바꾼 뒤 baseline(정지 시 vert)이 빠르게
// 0으로 수렴한다. 단, 스트럼(<0.3s) 가속이 중력에 너무 새어들지 않게 0.8s 정도로.
// (너무 크면 캘리 후 자세 변경 잔상이 오래 남아 가짜 vert가 생긴다.)
const float FB_GRAV_TAU = 0.8f;
// 폴백에서 가속도 ±8g로 설정하므로 1g ≈ 4096 LSB. (캘리브레이션 실측값으로 대체됨)
const int   FB_ACCEL_FS = 4096;

// ---------------------------------------------------------------------------
//  스트럼 트리거 출력 (악보 팔로우 모드)
// ---------------------------------------------------------------------------
//  펌웨어는 "스트럼이 한 번 일어났다 + 방향 + 세기"를 PC(midi.html)로 알린다.
//  실제 음표(코드)는 PC가 왼손 코드로 결정하고, 방향 일치 여부도 PC가 악보와 비교한다.
//  출력 형식 : "STRUM:<D|U>:<velocity>\n"  (D=다운, U=업, velocity 0~127)
// ---------------------------------------------------------------------------
void playStrum(bool isDown, int velocity) {
  Serial.print("STRUM:");
  Serial.print(isDown ? 'D' : 'U');
  Serial.print(':');
  Serial.println(velocity);
}

// ---------------------------------------------------------------------------
//  MPU / DMP 상태
// ---------------------------------------------------------------------------
MPU6500 mpu;

bool     useDMP = false;
bool     dmpReady = false;
uint8_t  devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t  fifoBuffer[64];

Quaternion  q;
VectorInt16 aa;        // DMP 패킷의 가속도 (LSB)
VectorFloat gravity;   // 중력 방향 단위벡터(바디 프레임)
float       ypr[3];    // [yaw, pitch, roll]

// 캘리브레이션 기준값
float orientRef = 0.0f; // 연주 자세에서의 기준 자세각(rad)
float gravLSB   = 4096.0f; // 정지 시 중력 크기(LSB) - 자동 측정해 스케일에 사용
int   strumTh, releaseTh, accelMax; // gravLSB 기반으로 환산한 가속 임계(LSB)

bool isStrumming = false;
unsigned long lastStrumMs = 0; // 마지막 스트럼 발사 시각 (불응기 판정용)
unsigned long fbQuietStart = 0; // vert가 조용해지기 시작한 시각 (재무장 판정용)
int lastStrumGyroSign = 0; // 직전 발사 스트로크의 스트럼축 자이로 부호(+1/-1), 0=없음
uint8_t g_whoami = 0; // WHO_AM_I(0x75) 값 - 부팅 시 1회 측정, 디버그 라인에 표시

// ---------------------------------------------------------------------------
//  공통 유틸
// ---------------------------------------------------------------------------
float wrapPi(float a) {
  while (a >  PI) a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

// ===========================================================================
//  DMP 경로
// ===========================================================================

// FIFO에서 가장 최신 패킷 한 개를 읽어 전역 상태(q, gravity, ypr, aa) 갱신.
// 새 데이터가 없으면 false. ESP32에서는 INT핀 없이 폴링으로 처리한다.
bool readDMP() {
  fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize) return false;

  // 오버플로(코드가 느려 FIFO가 꽉 찬 경우) -> 초기화 후 다음 루프
  if (fifoCount >= 1024 || (mpu.getIntStatus() & 0x10)) {
    mpu.resetFIFO();
    return false;
  }

  // 지연을 줄이기 위해 쌓인 패킷은 버리고 가장 최신 것만 사용
  while (fifoCount >= (uint16_t)(packetSize * 2)) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
  }
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  return true;
}

// 현재 패킷으로부터 중력축(수직) 선형가속도와 수평 성분 크기를 계산.
// vertAccel: 중력 방향 투영에서 중력 크기를 뺀 값(=위아래 선형가속, 부호 포함)
// horizMag : 중력에 수직인 성분의 크기(=좌우/앞뒤 움직임)
void computeMotion(float &vertAccel, float &horizMag) {
  float along = aa.x * gravity.x + aa.y * gravity.y + aa.z * gravity.z;
  vertAccel = along - gravLSB; // 정지 시 0이 되도록 중력 성분 제거
  float total2 = (float)aa.x * aa.x + (float)aa.y * aa.y + (float)aa.z * aa.z;
  float h2 = total2 - along * along;
  horizMag = (h2 > 0.0f) ? sqrt(h2) : 0.0f;
}

// 부팅 후 연주 자세로 정지한 상태에서 기준 자세각과 중력 크기를 측정.
void calibratePoseDMP() {
#ifdef DEBUG_TUNE
  Serial.println(F("[CAL] 연주 자세로 들고 2초간 정지하세요..."));
#endif
  // FIFO 안정화 대기
  unsigned long warm = millis();
  while (millis() - warm < 600) { readDMP(); }

  double sumOrient = 0.0, sumGrav = 0.0;
  long   samples = 0;
  unsigned long start = millis();
  while (millis() - start < 1500) {
    if (readDMP()) {
      sumOrient += ypr[ORIENT_AXIS];
      sumGrav   += (aa.x * gravity.x + aa.y * gravity.y + aa.z * gravity.z);
      samples++;
    }
  }

  if (samples > 0) {
    orientRef = (float)(sumOrient / samples);
    gravLSB   = (float)(sumGrav / samples);
  }
  if (gravLSB < 1000.0f) gravLSB = 4096.0f; // 비정상 측정 방어

  strumTh   = (int)(gravLSB * STRUM_G);
  releaseTh = (int)(gravLSB * RELEASE_G);
  accelMax  = (int)(gravLSB * ACCEL_MAX_G);

#ifdef DEBUG_TUNE
  Serial.print(F("[CAL] orientRef(deg)="));
  Serial.print(orientRef * 180.0f / PI, 1);
  Serial.print(F("  gravLSB="));
  Serial.print(gravLSB, 0);
  Serial.print(F("  strumTh="));
  Serial.println(strumTh);
#endif
}

void loopDMP() {
  if (!readDMP()) return;

  float vertAccel, horizMag;
  computeMotion(vertAccel, horizMag);
  float vertAbs  = fabs(vertAccel);
  float orientRel = wrapPi(ypr[ORIENT_AXIS] - orientRef);
  bool  poseOK   = fabs(orientRel) < (POSE_TOL_DEG * PI / 180.0f);

#ifdef DEBUG_TUNE
  static unsigned long lastDbg = 0;
  if (millis() - lastDbg > 120) {
    lastDbg = millis();
    Serial.print("rollRel(deg)=");
    Serial.print(orientRel * 180.0f / PI, 1);
    Serial.print("  vert=");
    Serial.print(vertAccel, 0);
    Serial.print("  horiz=");
    Serial.print(horizMag, 0);
    Serial.print("  pose=");
    Serial.print(poseOK ? "IN" : "out");
    Serial.println();
  }
#endif

  // 스트럼 시작 조건:
  //  (1) 연주 자세(손목 안쪽)이고
  //  (2) 수직 선형가속이 임계 초과이며
  //  (3) 위아래가 수평보다 우세 (= 아무 방향/수평 흔들림 배제)
  //  방향은 PC(악보)가 결정하므로 여기서는 세기(velocity)만 측정한다.
  //  (4) 직전 스트럼 후 불응기(STRUM_REFRACTORY_MS)가 지났을 때만 인정.
  if (!isStrumming && poseOK &&
      vertAbs > strumTh && horizMag < vertAbs * HORIZ_DOM_RATIO &&
      (millis() - lastStrumMs > STRUM_REFRACTORY_MS)) {

    // 온셋(임계 첫 돌파) 순간의 부호 = 스윙 의도 방향. +(중력 방향)=다운, -=업.
    bool isDown = (vertAccel > 0.0f);
#if INVERT_STRUM_DIR
    isDown = !isDown;
#endif

    float maxPeak = vertAbs;
    unsigned long startTime = millis();

    // 타격 최고점 추적 (세기 산출용)
    while (millis() - startTime < (unsigned long)PEAK_WINDOW_MS) {
      if (readDMP()) {
        float v, h;
        computeMotion(v, h);
        if (fabs(v) > maxPeak) maxPeak = fabs(v);
      }
    }

    int peak = constrain((int)maxPeak, strumTh, accelMax);
    int velocity = map(peak, strumTh, accelMax, 70, 127);

    playStrum(isDown, velocity);
    isStrumming = true;
    lastStrumMs = millis();
    mpu.resetFIFO(); // 트리거 처리 동안 쌓인 패킷 폐기 (오버플로/지연 방지)
  }

  // 쿨다운: 수직가속이 충분히 잦아들면 다음 스트럼 허용
  if (isStrumming && vertAbs < releaseTh) {
    isStrumming = false;
  }
}

// ===========================================================================
//  폴백 경로: 레지스터 직접 읽기 + 가속도 기반 중력벡터 추정
//  (DMP 경로와 동일한 게이팅: 손목 roll 자세 게이트 + 수직/수평 가속 분리)
// ===========================================================================
const int FB_MPU_ADDR = 0x68;

// 저역통과로 추정한 중력벡터(LSB, 바디 프레임)
float fbGravX = 0.0f, fbGravY = 0.0f, fbGravZ = 0.0f;
float fbRollRef = 0.0f, fbPitchRef = 0.0f;  // 연주 자세 기준 각(rad)
float fbGravLSB = (float)FB_ACCEL_FS; // 정지 시 중력 크기(LSB)
int   fbStrumTh, fbReleaseTh, fbAccelMax; // gravLSB 기반 환산 임계(LSB)
unsigned long fbLastMicros = 0;
int16_t fbGyX = 0, fbGyY = 0, fbGyZ = 0; // 최근 자이로(LSB) - 진단용

// 가속도(필수) + 자이로(진단용)를 한 번에 읽는다. 자이로는 전역에 저장.
void fbReadAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  // ESP32에서는 repeated-start(false)보다 STOP(true)이 안정적이라, 빠르게 연속
  // 호출해도 0xFFFF(-1) 쓰레기값이 잘 안 나온다.
  Wire.beginTransmission(FB_MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  // 읽기 실패 시 중력 추정값을 그대로 반환 -> 선형가속≈0 으로 계산돼 헛트리거를 막는다.
  // (0을 반환하면 ax-grav 가 커져 가짜 스윙으로 잡힘) gyro는 직전값 유지.
  if (Wire.endTransmission(true) != 0) {
    ax = (int16_t)fbGravX; ay = (int16_t)fbGravY; az = (int16_t)fbGravZ; return;
  }
  uint8_t got = Wire.requestFrom((uint16_t)FB_MPU_ADDR, (uint8_t)14, (bool)true);
  if (got < 14) {
    ax = (int16_t)fbGravX; ay = (int16_t)fbGravY; az = (int16_t)fbGravZ; return;
  }
  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // temp
  fbGyX = Wire.read() << 8 | Wire.read();
  fbGyY = Wire.read() << 8 | Wire.read();
  fbGyZ = Wire.read() << 8 | Wire.read();
}

// 저역통과 필터로 중력벡터 추정 갱신. 루프 속도와 무관하게 일정한 시간상수를
// 갖도록 실제 경과시간(dt)으로 계수를 계산한다. alpha = exp(-dt/tau)
void fbUpdateGravity(int16_t ax, int16_t ay, int16_t az) {
  unsigned long now = micros();
  float dt = (fbLastMicros == 0) ? 0.0f : (now - fbLastMicros) / 1000000.0f;
  fbLastMicros = now;
  float alpha = (dt > 0.0f) ? exp(-dt / FB_GRAV_TAU) : 1.0f;
  fbGravX = alpha * fbGravX + (1.0f - alpha) * ax;
  fbGravY = alpha * fbGravY + (1.0f - alpha) * ay;
  fbGravZ = alpha * fbGravZ + (1.0f - alpha) * az;
}

// 현재 추정 중력벡터로부터 roll / pitch (rad)
float fbRollNow()  { return atan2(fbGravY, fbGravZ); }
float fbPitchNow() { return atan2(-fbGravX, sqrt(fbGravY * fbGravY + fbGravZ * fbGravZ)); }

// 선택된 자세 축의 기준 대비 편차(rad). ORIENT_AXIS: 2=roll, 1=pitch
float fbOrientRel() {
  if (ORIENT_AXIS == 1) return wrapPi(fbPitchNow() - fbPitchRef);
  return wrapPi(fbRollNow() - fbRollRef);
}

// raw 가속도에서 중력을 빼고, 중력축 성분(수직)과 수평 성분을 분리
void fbComputeMotion(int16_t ax, int16_t ay, int16_t az,
                     float &vertAccel, float &horizMag) {
  float gm = sqrt(fbGravX * fbGravX + fbGravY * fbGravY + fbGravZ * fbGravZ);
  if (gm < 1.0f) gm = 1.0f;
  float ux = fbGravX / gm, uy = fbGravY / gm, uz = fbGravZ / gm;
  float lx = ax - fbGravX, ly = ay - fbGravY, lz = az - fbGravZ; // 선형가속(중력 제거)
  vertAccel = lx * ux + ly * uy + lz * uz;
  float lin2 = lx * lx + ly * ly + lz * lz;
  float h2 = lin2 - vertAccel * vertAccel;
  horizMag = (h2 > 0.0f) ? sqrt(h2) : 0.0f;
}

// 단일 레지스터 8비트 읽기 (진단용)
uint8_t fbReadReg(uint8_t reg) {
  // ESP32는 repeated-start(endTransmission(false)) 후 1바이트 읽기가 0을 반환하는
  // 에라타가 있어, STOP(true)을 써야 단일 바이트 레지스터가 정상적으로 읽힌다.
  Wire.beginTransmission(FB_MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(true);
  Wire.requestFrom((uint16_t)FB_MPU_ADDR, (uint8_t)1, (bool)true);
  return Wire.available() ? Wire.read() : 0x00;
}

void setupFallback() {
  Wire.beginTransmission(FB_MPU_ADDR);
  Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true); // 슬립 해제
  delay(10);

  // 풀스케일을 레지스터에 "직접" 써서 강제한다(라이브러리 writeBits 경유 시 이 보드에선
  // 적용이 안 돼 ±2g로 남아 강한 스트럼이 포화됐다). 포화는 vert/자이로를 모두 망쳐
  // 방향(D/U)까지 틀리게 만든다.
  //  GYRO_CONFIG(0x1B) bits[4:3]=FS_SEL: ±2000°/s = 0b11 -> 0x18
  //  ACCEL_CONFIG(0x1C) bits[4:3]=FS_SEL: ±8g     = 0b10 -> 0x10  (1g ≈ 4096 LSB)
  Wire.beginTransmission(FB_MPU_ADDR);
  Wire.write(MPU6500_RA_GYRO_CONFIG); Wire.write(0x18); Wire.endTransmission(true);
  Wire.beginTransmission(FB_MPU_ADDR);
  Wire.write(MPU6500_RA_ACCEL_CONFIG); Wire.write(0x10); Wire.endTransmission(true);
  delay(10);

#ifdef DEBUG_TUNE
  // 적용 확인: ACCEL_CONFIG=0x10, GYRO_CONFIG=0x18 이어야 한다.
  Serial.print(F("[CFG] GYRO_CONFIG=0x")); Serial.print(fbReadReg(MPU6500_RA_GYRO_CONFIG), HEX);
  Serial.print(F("  ACCEL_CONFIG=0x")); Serial.println(fbReadReg(MPU6500_RA_ACCEL_CONFIG), HEX);
#endif
}

void calibratePoseFallback() {
  // 중력 추정 초기값을 첫 측정으로 설정(0에서 시작하는 수렴 지연 방지)
  int16_t ax, ay, az;
  fbReadAccel(ax, ay, az);
  fbGravX = ax; fbGravY = ay; fbGravZ = az;

  unsigned long warm = millis();
  while (millis() - warm < 400) {
    fbReadAccel(ax, ay, az);
    fbUpdateGravity(ax, ay, az);
    delay(3);
  }

  // 정지 상태의 raw 가속도를 평균해 중력 방향(roll/pitch 기준)을 잡고,
  // gravLSB(1g 크기)는 "각 샘플 |A|의 평균"으로 구한다.
  //  - 벡터 성분 평균의 크기(√(avgX²+avgY²+avgZ²))는 캘리 중 살짝만 움직여도 성분이
  //    상쇄돼 작아진다(이전 버그: gravLSB가 16000이어야 하는데 2938로 나옴).
  //  - 반면 각 샘플의 크기 |A|는 정지 시 회전과 무관하게 항상 ≈1g 라 평균이 안정적.
  double sx = 0.0, sy = 0.0, sz = 0.0, smag = 0.0; long n = 0;
  unsigned long start = millis();
  while (millis() - start < 1500) {
    fbReadAccel(ax, ay, az);
    sx += ax; sy += ay; sz += az;
    smag += sqrt((double)ax * ax + (double)ay * ay + (double)az * az);
    n++;
    delay(5);
  }
  if (n > 0) { fbGravX = sx / n; fbGravY = sy / n; fbGravZ = sz / n; }

  fbGravLSB = (n > 0) ? (float)(smag / n) : (float)FB_ACCEL_FS;
  if (fbGravLSB < 1000.0f) fbGravLSB = (float)FB_ACCEL_FS; // 비정상 방어 (±8g: 1g≈4096)
  fbRollRef  = fbRollNow();
  fbPitchRef = fbPitchNow();

  fbStrumTh   = (int)(fbGravLSB * STRUM_G);
  fbReleaseTh = (int)(fbGravLSB * RELEASE_G);
  fbAccelMax  = (int)(fbGravLSB * ACCEL_MAX_G);

#ifdef DEBUG_TUNE
  Serial.print(F("[CAL-FB] rollRef(deg)="));
  Serial.print(fbRollRef * 180.0f / PI, 1);
  Serial.print(F("  pitchRef(deg)="));
  Serial.print(fbPitchRef * 180.0f / PI, 1);
  Serial.print(F("  gravLSB="));
  Serial.print(fbGravLSB, 0);
  Serial.print(F("  strumTh="));
  Serial.println(fbStrumTh);
#endif
}

void loopFallback() {
  int16_t ax, ay, az;
  fbReadAccel(ax, ay, az);
  fbUpdateGravity(ax, ay, az);

  float vertAccel, horizMag;
  fbComputeMotion(ax, ay, az, vertAccel, horizMag);
  float vertAbs = fabs(vertAccel);
#if USE_POSE_GATE
  bool poseOK = fabs(fbOrientRel()) < (POSE_TOL_DEG * PI / 180.0f);
#else
  bool poseOK = true; // 브링업: 자세 게이트 비활성
#endif

#ifdef DEBUG_TUNE
  static unsigned long lastDbg = 0;
  if (millis() - lastDbg > 120) {
    lastDbg = millis();
    // 어느 축으로 회전(스윙)하는지: 자이로 절댓값이 가장 큰 축을 표시
    long agx = labs(fbGyX), agy = labs(fbGyY), agz = labs(fbGyZ);
    char dom = (agx >= agy && agx >= agz) ? 'X' : (agy >= agz ? 'Y' : 'Z');
    long gMag = (long)sqrt((double)fbGyX * fbGyX + (double)fbGyY * fbGyY + (double)fbGyZ * fbGyZ);
    Serial.print("[FB] A(x,y,z)=");
    Serial.print(ax); Serial.print(","); Serial.print(ay); Serial.print(","); Serial.print(az);
    Serial.print("  G(x,y,z)=");
    Serial.print(fbGyX); Serial.print(","); Serial.print(fbGyY); Serial.print(","); Serial.print(fbGyZ);
    Serial.print("  grav=");
    Serial.print(fbGravX, 0); Serial.print(","); Serial.print(fbGravY, 0); Serial.print(","); Serial.print(fbGravZ, 0);
    Serial.print("  vert="); Serial.print(vertAccel, 0);
    Serial.print("  horiz="); Serial.print(horizMag, 0);
    Serial.print("  gMag="); Serial.print(gMag);
    Serial.print("  domGyro="); Serial.print(dom);
    Serial.print("  vTh="); Serial.print(fbStrumTh);
    Serial.println();
  }
#endif

  if (!isStrumming && poseOK &&
      vertAbs > fbStrumTh && horizMag < vertAbs * HORIZ_DOM_RATIO &&
      (millis() - lastStrumMs > STRUM_REFRACTORY_MS)) {
    // 스윙 동안 (1) 수직가속 최대치(=세기) 와 (2) 각속도가 가장 클 때의 자이로값을
    // 함께 추적한다. 방향은 가속도 부호가 아니라 "각속도 피크 순간의 자이로 부호"로
    // 정해 스윙 도중 부호 반전에 흔들리지 않게 한다.
    float maxPeak = vertAbs;
    long peakGyroMag2 = (long)fbGyX * fbGyX + (long)fbGyY * fbGyY + (long)fbGyZ * fbGyZ;
    int16_t pgx = fbGyX, pgy = fbGyY, pgz = fbGyZ;
    float peakLinY = (float)ay - fbGravY; // 자이로 피크 순간의 A(y) 선형성분(중력 제거)
    unsigned long startTime = millis();
    while (millis() - startTime < (unsigned long)PEAK_WINDOW_MS) {
      delay(2); // I2C를 과도하게 두들기지 않게 간격을 둔다(쓰레기 읽기 방지)
      int16_t bx, by, bz;
      fbReadAccel(bx, by, bz);          // fbGyX/Y/Z 동시 갱신
      fbUpdateGravity(bx, by, bz);
      float v, h;
      fbComputeMotion(bx, by, bz, v, h);
      if (fabs(v) > maxPeak) maxPeak = fabs(v);
      long gm2 = (long)fbGyX * fbGyX + (long)fbGyY * fbGyY + (long)fbGyZ * fbGyZ;
      if (gm2 > peakGyroMag2) { peakGyroMag2 = gm2; pgx = fbGyX; pgy = fbGyY; pgz = fbGyZ; peakLinY = (float)by - fbGravY; }
    }

    // 방향 자이로축 선택: 고정(STRUM_GYRO_AXIS>=0) 또는 피크에서 |값| 최대인 축(-1).
    int16_t dirVal;
    char dirAxis;
#if (STRUM_GYRO_AXIS == 0)
    dirVal = pgx; dirAxis = 'X';
#elif (STRUM_GYRO_AXIS == 1)
    dirVal = pgy; dirAxis = 'Y';
#elif (STRUM_GYRO_AXIS == 2)
    dirVal = pgz; dirAxis = 'Z';
#else
    { long apx = labs(pgx), apy = labs(pgy), apz = labs(pgz);
      if (apx >= apy && apx >= apz)      { dirVal = pgx; dirAxis = 'X'; }
      else if (apy >= apz)               { dirVal = pgy; dirAxis = 'Y'; }
      else                               { dirVal = pgz; dirAxis = 'Z'; } }
#endif
    // 방향 점수: 자이로(주) + A(y)(보조)를 각각 정규화해 가중 합. >0 이면 다운.
    //  gyroNorm : 선택축 자이로를 REF로 나눠 [-1,1] 클램프 (부호=회전방향)
    //  accNorm  : A(y) 선형성분을 REF로 나눠 클램프, DIR_ACC_SIGN으로 다운방향에 정렬
    float gyroNorm = constrain((float)dirVal / DIR_GYRO_REF, -1.0f, 1.0f);
    float dirScore = DIR_W_GYRO * gyroNorm;
#if DIR_USE_ACCEL
    float accNorm  = constrain((float)DIR_ACC_SIGN * peakLinY / DIR_ACC_REF, -1.0f, 1.0f);
    dirScore += DIR_W_ACC * accNorm;
#endif
    bool isDown = (dirScore > 0);
#if INVERT_STRUM_DIR
    isDown = !isDown;
#endif

    int peak = constrain((int)maxPeak, fbStrumTh, fbAccelMax);
    int velocity = map(peak, fbStrumTh, fbAccelMax, 70, 127);

#ifdef DEBUG_TUNE
    Serial.print("[dir] gyroPeak=(");
    Serial.print(pgx); Serial.print(","); Serial.print(pgy); Serial.print(","); Serial.print(pgz);
    Serial.print(")  axis="); Serial.print(dirAxis);
    Serial.print("  Gz="); Serial.print(pgz);
    Serial.print("  Ay_lin="); Serial.print(peakLinY, 0);
    Serial.print("  [Gz->"); Serial.print(pgz > 0 ? "+" : "-");
    Serial.print(" Ay->"); Serial.print(peakLinY > 0 ? "+" : "-");
    Serial.print("]  score="); Serial.print(dirScore, 2);
    Serial.print("  -> "); Serial.println(isDown ? "D" : "U");
#endif

    playStrum(isDown, velocity);
    isStrumming = true;
    lastStrumMs = millis();
    fbQuietStart = 0;
    lastStrumGyroSign = (dirVal > 0) ? 1 : -1; // 이번 스트로크의 스트럼축 자이로 부호
  }

  // 재무장: (a) 반대 스트로크 시작(연속 스트럼) 또는 (b) 동작이 충분히 잠잠(단발)
  if (isStrumming) {
    // 현재 스트럼축 자이로값
#if (STRUM_GYRO_AXIS == 0)
    int16_t gAxisNow = fbGyX;
#elif (STRUM_GYRO_AXIS == 1)
    int16_t gAxisNow = fbGyY;
#else
    int16_t gAxisNow = fbGyZ; // 2 또는 자동(-1) 모두 Z를 재무장 기준축으로 사용
#endif
    int curSign = (gAxisNow > 0) ? 1 : -1;
    bool reversed = (lastStrumGyroSign != 0) &&
                    (abs((int)gAxisNow) > STRUM_REVERSE_GYRO) &&
                    (curSign != lastStrumGyroSign);

    bool quiet = false;
    if (vertAbs < fbReleaseTh) {
      if (fbQuietStart == 0) fbQuietStart = millis();
      else if (millis() - fbQuietStart > STRUM_QUIET_MS) quiet = true;
    } else {
      fbQuietStart = 0; // 다시 움직임 -> 조용 타이머 리셋
    }

    if (reversed || quiet) { isStrumming = false; fbQuietStart = 0; }
  }
}

// ===========================================================================
//  SETUP / LOOP
// ===========================================================================
void setup() {
  Serial.begin(115200);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin(I2C_SDA, I2C_SCL);
#endif

#ifdef DEBUG_TUNE
  Serial.println(F("Initializing MPU6500..."));
#endif
  mpu.initialize();

  // WHO_AM_I(레지스터 0x75) — 칩 정체 확인용(부팅 시 1회). ESP32에서는 STOP 방식으로
  // 읽어야 단일 바이트가 정상 반환된다(0x70=MPU6500).
  for (int i = 0; i < 30; i++) {
    uint8_t v = fbReadReg(0x75);
    if (v != 0x00 && v != 0xFF) { g_whoami = v; break; }
    delay(5);
  }
#ifdef DEBUG_TUNE
  Serial.print(F("WHO_AM_I (0x75) = 0x"));
  Serial.println(g_whoami, HEX);
#endif

#if USE_DMP
  // DMP 경로 (우노 등 DMP 동작 보드 전용). ESP32에서는 USE_DMP=0이라 컴파일되지 않는다.
  devStatus = 1;
  const uint32_t i2cClocks[] = {400000, 200000, 100000};
  for (int attempt = 0; attempt < 3 && devStatus != 0; attempt++) {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.setClock(i2cClocks[attempt]);
#endif
    devStatus = mpu.dmpInitialize();
#ifdef DEBUG_TUNE
    Serial.print(F("dmpInitialize @"));
    Serial.print(i2cClocks[attempt]);
    Serial.print(F("Hz -> code="));
    Serial.println(devStatus);
#endif
    if (devStatus == 0) break;
    delay(80);
  }
#else
  devStatus = 1; // DMP 미사용 -> 폴백 강제
#endif

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    mpu.resetFIFO();
    useDMP = true;
    dmpReady = true;
#ifdef DEBUG_TUNE
    Serial.println(F("DMP ready (FIFO polling mode)."));
#endif
    calibratePoseDMP();
  } else {
    // 폴백(레지스터 직접 읽기) 모드 — ESP32 기본 경로.
    useDMP = false;
#ifdef DEBUG_TUNE
    Serial.println(F("Fallback mode (register read)."));
#endif
    setupFallback();
    calibratePoseFallback();
  }
}

void loop() {
  if (useDMP) {
    if (!dmpReady) return;
    loopDMP();
  } else {
    loopFallback();
  }
}
