# 🎸 Wearable Guitar (웨어러블 기타)

웨어러블 디바이스를 통해 손동작과 스마트폰 터치로 실제 기타를 연주하듯 소리를 낼 수 있는 가상 악기 프로젝트입니다. 왼손으로는 스마트폰 화면(혹은 차후 물리적인 버튼이 부착된 NFT 케이스)을 터치하여 코드를 잡고, 오른손으로는 IMU 센서가 장착된 디바이스를 흔들어 스트럼(Strumming) 모션으로 소리를 냅니다.

> **연주 방식: 악보 팔로우(Score-Follow)**
> 손목 각도를 정밀하게 바꾸기 어렵다는 점을 반영해, 오른손은 **위아래로 스트럼만 반복**하면 됩니다. 업/다운 방향과 진행은 미리 입력한 **악보(스트럼 패턴, 예: `U U D U`)** 가 결정하고, 소리나는 화음은 **왼손이 실시간으로 잡은 코드**가 결정합니다. 즉 **스트럼 1회 = 악보 한 칸 진행**(타이밍 판정 없는 follow 방식)입니다.

---

## 🛠️ 시스템 구성 (System Architecture)

### 1. 왼손 (Left Hand) - 코드 컨트롤러
* **역할**: 연주할 화음(Chord) 선택 및 제어
* **구성**: 스마트폰 웹 브라우저 앱 (차후 NFT 전용 피팅 케이스를 활용한 그립 및 버튼 입력 예정)
* **동작**: 모바일 화면에서 코드 버튼(A~F Major)을 터치하거나 슬라이드하면 WebSocket을 통해 실시간으로 입력 이벤트를 송신합니다.

### 2. 오른손 (Right Hand) - 스트럼 트리거
* **역할**: "스트럼이 한 번 일어났다"는 신호만 발생 (음표/코드/방향은 PC가 결정)
* **구성**: ESP32 Lotin Lite, MPU-6500 IMU(관성 측정 장치) 센서
* **동작**: 중력벡터 대비 **수직(위아래) 선형가속**이 임계를 넘고 수평 흔들림보다 우세할 때만 스트럼으로 인정합니다(아무 방향/수평 흔들림 배제). 스트럼이 감지되면 타격 세기만 측정해 시리얼로 **`STRUM:<velocity>`** 한 줄을 내보냅니다.
  * 코드 테이블·MIDI 음 생성은 더 이상 펌웨어에 없습니다. 펌웨어는 순수 "스트럼 트리거 센서"입니다.
  * **동작 모드: 가속도 기반 폴백.** MPU-6500 내장 DMP는 이 모듈+우노 조합에서는 정상 동작하지만(칩은 정품 확인됨, WHO_AM_I=0x70), **ESP32에서는 DMP 펌웨어 전송 중 I2C 크래시/버스행으로 실패**합니다. 실패한 DMP 시도가 버스까지 불안정하게 만들어, ESP32 펌웨어는 `USE_DMP 0`으로 **DMP를 건너뛰고 폴백으로 곧장 부팅**합니다. 폴백은 가속도로 중력벡터를 추정해 수직 스윙을 감지하므로 스트럼 감지 정확도는 충분합니다. (DMP가 동작하는 보드로 옮길 경우 `USE_DMP 1`)

### 3. PC 브릿지 (악보 팔로워) & 가라지밴드
* **역할**: 왼손 코드 + 악보 + 오른손 스트럼을 합쳐 실제 MIDI 음을 생성하고 GarageBand로 전송
* **구성**: Chrome 브라우저의 Web Serial / Web MIDI / WebSocket API (`midi(right)/midi.html`)
* **동작**: midi.html이 시스템의 **허브**입니다.
  1. 왼손 앱이 보낸 코드를 WebSocket으로 받아 **현재 코드**를 기억합니다.
  2. 입력한 악보(예: `D U D U`)를 스텝 배열로 보관하고 **현재 위치**를 추적합니다.
  3. ESP32가 `STRUM:<velocity>` 를 보낼 때마다 [현재 코드 × 악보의 현재 스텝 방향]으로 아르페지오 음을 만들어 **IAC 드라이버**를 통해 GarageBand로 전송하고, 악보를 한 칸 진행(기본 반복)합니다.
  * 다운(`D`)은 저음→고음, 업(`U`)은 고음→저음 순으로 음을 펼칩니다. 악보가 비어 있으면 매 스트럼을 다운으로 연주합니다.

---

## 📂 디렉토리 구조 (Directory Structure)

```text
Wearable Guitar/
├── README.md                    # 프로젝트 전체 가이드 (본 파일)
├── Guitar_right_light/          # [오른손] 스트럼 트리거 ESP32 펌웨어
│   ├── Guitar_right_light.ino   # 메인 스케치 (스트럼 온셋 감지 → STRUM:vel 출력 + 캘리브레이션 + 폴백)
│   └── I2Cdev / MPU6500 / MotionApps41 / helper_3dmath  # DMP 라이브러리 (rabbit에서 동봉)
│
├── chord-controller(left)/      # [왼손] 코드 제어 모바일 웹앱 및 Node.js 서버 (포트 8080)
├── chord-controller-grid(left)/ # [왼손] 그리드 UI 버전 (포트 8082)
│   ├── server.js                # WebSocket 및 정적 웹 서비스 제공을 위한 Node Server
│   ├── package.json             # 종속성 관리 파일
│   └── public/                  # 스마트폰 전용 터치 UI 리소스 (HTML, CSS, JS)
│
├── midi(right)/                 # [PC 허브] 악보 팔로워 + 시리얼/MIDI/WS 브릿지
│   └── midi.html                # 코드 수신 + 악보 진행 + 아르페지오 MIDI 생성 페이지
│
└── mpu6500_dmp6_rabbit/         # MPU-6500 정밀도 향상을 위한 DMP(Digital Motion Processor) 프로젝트
    ├── mpu6500_dmp6_rabbit.ino  # 쿼터니언 출력 기반 펌웨어 (교수님 연구 프로젝트)
    └── (I2Cdev 및 MPU6500 라이브러리 소스코드 포함)
```

---

## 🚀 동작 및 사용 방법 (How to Run)

### Step 1. 오른손 하드웨어 준비 및 펌웨어 업로드
1. **ESP32 Lotin Lite** 보드와 **MPU-6500** 센서를 I2C로 연결합니다.
   * **I2C 핀 맵**: SDA ➡️ GPIO 19, SCL ➡️ GPIO 22 (INT핀은 사용하지 않으며 FIFO 폴링 방식)
2. Arduino IDE에서 `Guitar_right_light/` 폴더(스케치 + 동봉된 I2Cdev/MPU6500/MotionApps41/helper_3dmath 라이브러리)를 열어 보드에 업로드합니다.
3. **부팅 직후 약 2초간** 디바이스를 **연주 자세로 들고 정지**합니다. 이때 측정한 중력 방향이 기준값으로 저장됩니다(수직/수평 분리 기준).
   * 펌웨어는 `STRUM:<velocity>` 한 줄만 시리얼(115200)로 내보냅니다. 임계값 튜닝이 필요하면 스케치 상단의 `// #define DEBUG_TUNE` 주석을 해제해 `vert / horiz / gyro` 값을 보며 `STRUM_G`, `HORIZ_DOM_RATIO` 등을 조정합니다(평소엔 다시 주석 처리). DEBUG 라인을 켜도 `midi.html` 파서는 `STRUM:`으로 시작하는 줄만 처리하므로 동작에는 영향이 없습니다.
   * ESP32에서는 DMP 펌웨어 전송이 I2C 크래시/버스행으로 실패하므로 펌웨어가 `USE_DMP 0`으로 **폴백 모드(가속도 기반)** 로 곧장 부팅합니다. **가속도로 중력벡터를 추정해 수직/수평 가속을 분리**하므로 스트럼 감지는 안정적으로 동작합니다. (참고: 같은 MPU-6500 모듈도 우노에서는 DMP가 정상 동작 → 칩 자체는 정품이며, ESP32의 I2C 궁합 문제입니다. 단일 바이트 레지스터 읽기는 `I2Cdev`를 ESP32에서 STOP 방식으로 읽도록 수정해 해결했습니다.)

### Step 2. 왼손 코드 컨트롤러 서버 실행
1. `chord-controller(left)` 폴더로 이동하여 필요 패키지를 설치하고 서버를 실행합니다.
   ```bash
   cd chord-controller\(left\)
   npm install
   npm start
   ```
2. 같은 와이파이(Wi-Fi) 망에 연결된 스마트폰으로 아래 주소에 접속합니다. (포트는 실행한 서버 기준: 기본 `chord-controller(left)`=8080, 그리드 버전=8082)
   ```text
   http://<PC의_로컬_IP>:8082
   ```
   *(터치 스크린을 통해 코드를 선택할 수 있는 화면이 나타납니다.)*

### Step 3. PC 허브(midi.html) 설정
1. Mac에서 **오디오 MIDI 설정** 앱을 실행한 후, `윈도우 > MIDI 스튜디오 표시`를 클릭합니다.
2. **IAC 드라이버**를 더블 클릭한 뒤 **'장치가 온라인 상태임'**에 체크하여 가상 MIDI 버스를 활성화합니다.
3. Chrome 브라우저로 `midi(right)/midi.html` 파일을 엽니다.
4. 웹페이지에서 다음을 설정합니다.
   * **`1. 오른손(ESP32) 연결`**: ESP32가 연결된 시리얼 포트(115200 Baud)를 선택해 연결합니다.
   * **`2. 소리(MIDI) 연결`**: 누르면 사용 가능한 MIDI 출력이 **드롭다운**에 나열되고, 가장 적합한 출력(mac=IAC, Windows=loopMIDI/GS Wavetable)이 자동 선택됩니다. 필요하면 드롭다운에서 직접 바꿉니다.
   * **WebSocket URL**: 왼손 서버 포트와 맞춥니다(기본 `ws://localhost:8082`). `연결` 버튼을 누르면 왼손 코드가 "현재 코드"에 표시됩니다.
   * **악보**: 스트럼 패턴을 입력하고(예: `D U D U D U D U`) `적용`을 누릅니다. `반복` 체크 시 끝에서 처음으로 돌아가고, `처음으로`로 위치를 리셋합니다.

### Step 4. 가라지밴드(GarageBand) 연주
1. **GarageBand**를 실행하고 소프트웨어 악기(예: Acoustic Guitar 등) 트랙을 생성합니다.
2. 왼손으로 스마트폰 앱에서 원하는 코드를 짚고, 오른손 디바이스를 **위아래로 스트럼만 반복**합니다.
3. 스트럼할 때마다 악보의 현재 스텝(`D`/`U`) 방향으로, 왼손이 잡은 코드가 한 칸씩 진행되며 연주됩니다. midi.html의 악보 표시에서 초록색으로 강조된 칸이 다음에 연주될 스텝입니다.

---

## 🪟 Windows에서 실행 (GarageBand 없이)

`midi.html`은 크로스플랫폼입니다. GarageBand(=MIDI를 소리로 바꿔주는 신스)는 필수가 아니며, Windows에서는 아래 중 하나로 대체합니다. 펌웨어·서버 코드는 **수정 없이** 그대로 동작합니다.

### 필수
* **브라우저**: Chrome 또는 Edge (Web Serial / Web MIDI 필요 — Firefox/Safari 불가).
* **Node.js**: 왼손 서버(`npm install` / `npm start`)는 Windows에서 동일하게 동작합니다.
* **USB 시리얼 드라이버**: ESP32가 인식되지 않으면 보드 칩에 맞는 드라이버 설치 (CP210x 또는 CH340).
* **로컬 IP 확인**: `ipconfig` (mac의 `ipconfig getifaddr en0` 대신).

### 소리 출력 선택 (셋 중 하나)
| 방법 | 난이도 | 설치 | 비고 |
|---|---|---|---|
| **내장 Microsoft GS Wavetable Synth** | 가장 쉬움 | 없음 | "소리(MIDI) 연결" 드롭다운에서 바로 선택. 약간의 지연·기본 음색 |
| **VirtualMIDISynth + 사운드폰트** | 보통 | [VirtualMIDISynth](https://coolsoft.altervista.org/en/virtualmidisynth) + .sf2 | 더 좋은 음색 |
| **loopMIDI + DAW**(Reaper/FL 등) | 높음 | [loopMIDI](https://www.tobias-erichsen.de/software/loopmidi.html) + DAW | 최고 음질·VST 사용 |

### 순서
1. (선택) 위 표의 가상 MIDI 포트/신스를 설치합니다. 가장 빠르게 하려면 설치 없이 **GS Wavetable**을 그대로 씁니다.
2. 왼손 서버 실행: `cd chord-controller(left)` → `npm install` → `npm start`.
3. Chrome/Edge로 `midi(right)/midi.html`을 엽니다.
4. **`2. 소리(MIDI) 연결`** → 드롭다운에서 출력(GS Wavetable / loopMIDI 등)을 선택합니다.
5. 나머지(시리얼 연결, WebSocket, 모드 선택)는 mac과 동일합니다.

> ※ 리버브(MIDI CC91)는 선택한 출력 뒤의 신스가 지원할 때만 적용됩니다(GS Wavetable·다수 사운드폰트는 지원).

---

## 🎯 향후 병합 및 개선 목표 (Roadmap)

1. **왼손-오른손-악보 연동 (악보 팔로우 통합 완료)**
   * 왼손 코드(WebSocket) · 오른손 스트럼(시리얼) · 악보(스트럼 패턴)를 `midi.html` 허브에서 합쳐, 스트럼할 때마다 [현재 코드 × 악보 스텝 방향]으로 GarageBand에 음을 전달하도록 통합했습니다.
   * 음 생성 주체를 ESP32 → PC로 옮겨, 펌웨어는 `STRUM:vel` 트리거만 보내는 단순 센서가 되었습니다.

2. **스트럼 감지 단순화 (악보가 방향을 결정)**
   * 손목 자세로 방향/연주 의도를 판별하는 대신, 방향은 악보가 정하고 펌웨어는 "수직 스윙이 일어났다"만 감지합니다(수평/임의 흔들림은 여전히 임계·우세도로 배제).
   * 남은 과제: 실제 착용 상태의 임계값(`STRUM_G`, `HORIZ_DOM_RATIO`) 현장 튜닝, 악보 파일(.txt) 업로드/리듬 동기화 모드 등 확장.
