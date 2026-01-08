| [Korean 🇰🇷](#korean) | [Japanese 🇯🇵](#japanese) | [Team](#Team) |
| :---: | :---: | :---: |

</div>

---

<div id="korean">

### 🇰🇷 Korean Version

# 🔋 E.S.S.E.N.T.I.A.L
## ESS Safety System with Environmental Network & Thermal Intelligent ALert Logic
ROS2 로봇 순찰과 센서 네트워크 데이터를 기반으로 이상 상황을 자동 감지·분석·대응하는 ESS 통합 안전 관제 솔루션

---

## 💡 1. 프로젝트 개요

본 프로젝트는 ESS(에너지 저장 장치) 시설의 화재, 가스 누출, 환경 변화 등 이상 징후를 **실시간으로 감지하고 대응**하기 위한 통합 안전 관제 시스템입니다.

STM32 센서 모듈과 ROS2 기반 순찰 로봇에서 수집된 데이터를 MQTT를 통해 통합하고, MariaDB에 기록하여 <strong>Qt 기반의 중앙 관제 센터(Control Tower)</strong>에서 실시간 모니터링과 원격 제어를 수행할 수 있도록 구축했습니다.

특히, 이동형 로봇과 고정형 센서를 결합한 **하이브리드 관제 아키텍처**를 통해 기존 고정형 센서만으로는 사각지대가 발생하던 문제를 해결했습니다.

---

### 🧩 시스템 아키텍처 (System Architecture)
**Edge Layer:** STM32 (환경 센서, Fan 제어), ROS2 Robot (열화상 탐지 및 순찰)  
**Communication:** MQTT (JSON Protocol)  
**Control Tower:** Qt C++ GUI Application, MariaDB

---

## 🛠️ 2. 기술 스택

### 언어
![C++](https://img.shields.io/badge/Language-C++-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)
![Python](https://img.shields.io/badge/Language-Python-3776AB?style=for-the-badge&logo=python&logoColor=white)
![SQL](https://img.shields.io/badge/Language-SQL-003545?style=for-the-badge&logo=mysql&logoColor=white)

### 프레임워크 / 라이브러리
![Qt](https://img.shields.io/badge/Framework-Qt6-41CD52?style=for-the-badge&logo=qt&logoColor=white)

### 통신
![MQTT](https://img.shields.io/badge/Protocol-MQTT-660066?style=for-the-badge)
![JSON](https://img.shields.io/badge/Format-JSON-F2C94C?style=for-the-badge)
![Wi-Fi](https://img.shields.io/badge/Protocol-ESP8266%20WiFi-FF6F61?style=for-the-badge)

### 데이터베이스
![MariaDB](https://img.shields.io/badge/DB-MariaDB-003545?style=for-the-badge&logo=mariadb&logoColor=white)

### 하드웨어 / 로봇
![STM32](https://img.shields.io/badge/MCU-STM32-3A5A99?style=for-the-badge)
![ROS2](https://img.shields.io/badge/Robot-ROS2-339933?style=for-the-badge)
![RaspberryPi](https://img.shields.io/badge/Platform-Raspberry%20Pi-CC0000?style=for-the-badge&logo=raspberrypi&logoColor=white)

### 센서 / 입력
![DHT11](https://img.shields.io/badge/Sensor-DHT11-56CCF2?style=for-the-badge)
![SGP30](https://img.shields.io/badge/Sensor-SGP30-F2994A?style=for-the-badge)
![MLX90640](https://img.shields.io/badge/Sensor-MLX90640-EB5757?style=for-the-badge)
![Camera](https://img.shields.io/badge/Sensor-RPi%20Camera-FFCA28?style=for-the-badge)
![RFID](https://img.shields.io/badge/Input-RFID-6FCF97?style=for-the-badge)


---

## 🎯 3. 핵심 기능

- **실시간 환경 감시 및 제어**
  - 구역별 온도·습도·가스 농도 모니터링
  - 임계치 도달 시 Fan 자동 가동 및 관제 UI에 상태 표시
  - Fan 제어 사유를 기록하여 이력 관리

- **열화상 기반 비상 탐지**
  - ROI(관심 영역) 분석을 통해 배터리 랙의 최고 온도와 좌표 추출
  - Critical 이상 발생 시 MQTT를 통한 즉시 알람 발행 및 DB 기록

- **ROS2 기반 자율 순찰 및 Auto-Docking**
  - NAV2를 활용한 지정 구역 순찰
  - ArUco 마커 인식으로 충전 위치 및 홈 복귀 정밀 보정

- **통합 관제 타워(Control Tower UI)**
  - 구역별 상태(Normal / Warning / Critical) 실시간 시각화
  - 배터리 랙 3x3 구조 시각화 위젯
  - 기간별 환경 데이터 및 출입·알람 로그 조회

---

## 👨‍💻 4. 역할 및 기여

- **Qt 관제 UI 개발**
  - 전체 대시보드 설계 및 배터리 랙/환경맵 위젯 구현
  - Custom Widget 설계 및 resizeEvent 최적화

- **MariaDB 스키마 설계**
  - 환경 데이터, 출입 로그, 이벤트 기록을 정규화하여 효율적 저장 구조 설계
  - 데이터 중복 방지를 위한 Fingerprint 로직 적용

- **MQTT 통합 데이터 핸들링**
  - Python Subscriber 구현으로 센서·로봇 데이터를 실시간 DB 연동
  - 데이터 정합성 및 누락/중복 방지 로직 적용

- **원격 제어 인터페이스**
  - Fan 및 공조 시스템 상태 모니터링·수동 제어
  - 제어 사유 기록 및 UI 반영

---

## 🐞 5. 트러블슈팅

### 1) 알람 종료 후 UI 상태 미복귀
- **현상**: 가스/열 이상 종료 후에도 UI 구역 색상 정상으로 돌아오지 않음
- **분석**: 이벤트 발생 시점만 처리하고 종료 기준 부재
- **해결**: SQL `DATE_SUB(NOW(), INTERVAL 10 SECOND)` 조건과 QSet 목록화로 Auto-Recovery 로직 구현
- **결과**: 수동 조작 없이 UI와 현장 상황 정합성 확보

### 2) MQTT 재접속 시 중복 데이터 적재
- **현상**: 서버 재시작 시 브로커에 남아있던 메시지가 DB에 중복 저장
- **분석**: clean_session 옵션 미설정으로 잔류 세션 데이터 발송
- **해결**: clean_session=True 적용 및 Fingerprint 기반 중복 저장 차단
- **결과**: DB 부하 최소화, 데이터 무결성 강화

### 3) 가변 창 크기 대응 시 위젯 위치 문제
- **현상**: Qt 창 크기 조절 시 배경 위 배터리 랙 위젯 위치 어긋남
- **분석**: 절대 좌표 기반 렌더링으로 리사이징 대응 불가
- **해결**: resizeEvent에서 부모 rect() 기반 overlay->setGeometry 동적 계산
- **결과**: 다양한 화면 환경에서 반응형 UI 구현

---

## 📚 6. 배운 점

- **엔드투엔드(End-to-End) 시스템 통합 역량 강화**  
  STM32 임베디드 제어기와 ROS2 로봇 플랫폼 간의 이기종 데이터를 MQTT로 통합하고, 이를 MariaDB 및 Qt UI까지 실시간으로 연결하는 전체 서비스 아키텍처를 설계하며 시스템 통합(SI) 프로세스 전반에 대한 실무 경험을 획득함.

- **데이터 무결성 및 시스템 견고성 설계**  
  네트워크 불안정 시 중복 적재를 방지하는 Fingerprint 알고리즘과, 누락된 이벤트를 자동 복구하는 SQL 기반 Auto-Recovery 로직을 설계함으로써, 실제 운영 환경에서 발생 가능한 예외 상황에 대한 시스템 대응 역량을 강화함.

- **실시간 데이터 파이프라인 최적화 경험**  
  MQTT 브로커와 Python Subscriber를 연동하여 지연(Latency)을 최소화한 실시간 데이터 파이프라인을 구축. 특히 비동기 처리를 통해 센서 데이터 수집부터 관제 화면 표출까지의 데이터 정합성을 확보함.

- **하이브리드 관제 아키텍처를 통한 사각지대 해소**  
  고정형 센서의 공간적 한계와 이동형 로봇의 시간적 제약을 상호 보완하는 하이브리드 구조를 구현. 이를 통해 ESS 시설 내 사각지대를 최소화하고, 다각도에서 이벤트를 분석·처리하는 로직 설계 경험을 습득함.

- **자원 효율적인 백엔드 데이터 관리**  
  제한된 하드웨어 자원 환경에서 성능을 유지하기 위해, 유의미한 상태 변화 데이터만을 선별 저장하도록 설계. 이를 통해 DB I/O 부하를 최적화하고 관리 로그 데이터의 가독성과 분석 효율성을 향상시킴.

---

<div align="center">
<a href="#japanese">⬇️ 日本語バージョンへ移動 (Go to Japanese Version) ⬇️</a>
</div>

</div>

---

<div id="japanese">

### 🇯🇵 Japanese Version

# 🔋 E.S.S.E.N.T.I.A.L
## ESS Safety System with Environmental Network & Thermal Intelligent ALert Logic
データ駆動型ESS統合安全監視システム：ROS2ロボット巡回とセンサーネットワークによる異常の自動検知・分析・対応



---

<div align="center">
<a href="#korean">⬆️ 한국어 버전으로 돌아가기 (Go back to Korean Version) ⬆️</a>
</div>

</div>

---
---

<div align="center">
<a href="#Team">⬇️ Go to Team Version ⬇️</a>
</div>

</div>

---

<div id="Team">

### ⏰ Team

# E.S.S.E.N.T.I.A.L  
**ESS Safety System with Environmental & Thermal Intelligent ALert**

> ESS 시설의 화재/가스/환경 이상을 감지하고,  
> Hybrid Patrol Robot(ROS2) + MQTT + Control Tower(서버/DB/UI)로 **즉시 알림/이력/관제**까지 이어지는 통합 안전 시스템

![title](docs/assets/slides/title.png)

---


### Full Demo (All-in-one)
<a href="docs/assets/video/full_demo.gif">
  <img src="docs/assets/video/full_demo.gif" width="900">
</a>

### Video Clips

<table>
  <tr>
    <td align="center">
      <a href="docs/assets/video/aruco.gif">
        <img src="docs/assets/video/aruco.gif" width="260">
      </a><br/>
      <b>ArUco Auto-Docking</b>
    </td>
    <td align="center">
      <a href="docs/assets/video/emergency.gif">
        <img src="docs/assets/video/emergency.gif" width="260">
      </a><br/>
      <b>Emergency</b>
    </td>
    <td align="center">
      <a href="docs/assets/video/critical.gif">
        <img src="docs/assets/video/critical.gif" width="260">
      </a><br/>
      <b>Critical</b>
    </td>
  </tr>
  <tr>
    <td align="center">
      <a href="docs/assets/video/warning.gif">
        <img src="docs/assets/video/warning.gif" width="260">
      </a><br/>
      <b>Warning</b>
    </td>
    <td align="center">
      <a href="docs/assets/video/ventilation.gif">
        <img src="docs/assets/video/ventilation.gif" width="260">
      </a><br/>
      <b>Ventilation</b>
    </td>
    <td align="center">
      <a href="docs/assets/video/rfid.gif">
        <img src="docs/assets/video/rfid.gif" width="260">
      </a><br/>
      <b>RFID</b>
    </td>
  </tr>
</table>

## Background
- ESS 시설의 주요 리스크(화재/가스/환경 변화)에 대해 **“감지 → 대응 → 확인/관제”**까지 연결된 시스템 필요
- 이 프로젝트는 Zone별 센서 데이터 수집과 로봇 순찰을 결합해, 이상 상황을 서버로 전송하고 DB 이력화 및 UI로 관제하는 것을 목표로 함

---

## Goals
- [ ] 각 Zone별 독립적 환경 데이터 수집
- [ ] MQTT 기반 초저지연 데이터 전송
- [ ] 위험 감지 시 이벤트/이력 저장 + 관제 UI 표시
- [ ] 로봇 순찰 + ArUco 기반 홈 복귀

---

## Hardware Architecture

<p align="center">
  <img src="docs/assets/slides/hw-arch.png" width="900" alt="HW Arch 1">
</p>
<p align="center">
  <img src="docs/assets/slides/hw-arch2.png" width="900" alt="HW Arch 2">
</p>
<p align="center">
  <img src="docs/assets/slides/hw-arch3.png" width="900" alt="HW Arch 3">
</p>

---

## Software Architecture
![sw-arch](docs/assets/slides/sw-arch.png)

---

## Data Flow / Sequence
![sequence](docs/assets/slides/sequence.png)

---

### ROS Nodes / Graph
![ros-nodes-1](docs/assets/diagrams/ros-nodes-1.png)
![ros-nodes-2](docs/assets/diagrams/ros-nodes-2.png)

## Key Features
### 1) Access Authentication & Alarm
- RFID 기반 관리자 인증
- 가스 이상 감지 시 시청각 알림 + 서버 즉각 전송(MQTT)

### 2) Environmental Monitoring & HVAC Control
- 온/습도 수집 및 임계 조건 기반 공조 제어
- 데이터는 DB 적재 + UI 조회/그래프 제공


### 3) Thermal Safety (Infrared)
- 열화상 ROI 기반 이상 픽셀 감지
- 최고온도/좌표 기반 이벤트 생성 → MQTT 전송 → DB 기록

### 4) Hybrid Patrol Robot (ROS2)
- NAV2 기반 순찰 + 구역/층 전환 로직
- ArUco 마커 기반 홈 위치 정렬(복귀 보정)

---

## MQTT Protocol (Topics)
> 서버 구독(예): `ess/env`, `ess/alert`, `ess/access/request`

### Environment
Topic: `ess/env`
```json
{"t": 23.10, "h": 55.30, "fan": "ON", "reason": "TEMP"}
```

### Alert (Gas / Thermal)
Topic: ess/alert
```text
{
  "event_type": "gas",
  "level": "warning",
  "value": 650,
  "location": "zone_1",
  "message": "Gas level elevated"
}
```

### Access Request/Response
Req Topic: ess/access/request
```text
{"admin_id":"RFID_123456","access_point":"main"}
```

Resp Topic: ess/access/response
```text
{"result":"success"}
```

### Project Structure
```text
.
├── deploy/                 # systemd/udev/scripts (로봇/라즈베리파이 자동실행)
├── ess_map/                # NAV2 map
└── src/
    ├── ess_server/         # MariaDB + MQTT subscriber + Qt UI
    ├── ess_control_pkg/    # ROS2 control node (NAV2/상태머신 등)
    ├── ess_mqtt_bridge_pkg/# ROS↔MQTT 브릿지(초안/실험 포함)
    ├── esp8266/            # ESP8266(D1 mini) MQTT bridge
    └── SGP30_3*/           # STM32 펌웨어(센서/제어)
```

### Deployment (systemd / udev)

deploy/ 폴더는 로봇(또는 Pi)에서 부팅 시 자동으로 서비스가 올라오도록 구성되어 있음.

udev: 카메라 심볼릭 링크(/dev/cam_rgb) 등

systemd:

ess-usb-camera.service : ROS2 카메라 퍼블리셔

ess-aruco-move.service : ArUco 기반 정렬/복귀 노드

ess-thermal-*.service : 열화상 체크/수집/게이트(환경에 따라 경로 조정 필요)

환경별로 /opt/ess-guardian/current/... 같은 경로는 수정이 필요할 수 있음.

### Troubleshooting (Short)

STM32 하드웨어 배선 이슈: 접점/전원/그라운드 재정리로 안정화

소프트 I2C Bit-banging 이슈: 타이밍 마진 조정 + 풀업/노이즈 대응

ROS NAV2 이슈: TF/파라미터 튜닝으로 주행 안정화

UI Update 이슈: 알림 해제 후 상태 전이 로직 보강 필요

<details>
<summary><b>Troubleshooting Screenshots</b></summary>

![hw-trouble](docs/assets/troubleshooting/hw-troubleshooting.png)
![ros-trouble-1](docs/assets/troubleshooting/ros-troubleshooting-1.png)
![ros-trouble-2](docs/assets/troubleshooting/ros-troubleshooting-2.png)
![ui-trouble](docs/assets/troubleshooting/ui-troubleshooting.png)

</details>

### Roadmap

이벤트 정합성(중복/쿨다운) 정책 고도화

Thermal ROI / 임계값 튜닝 자동화

Control Tower 기능 강화(필터/리포트/통계)

## 👥 Team & Roles

| Name | Role | Main Contribution (1-line) |
| :---: | :---: | :--- |
| 김찬미 (Team Leader) | STM32 / HW | 센서 수집 + 제어 로직 + 하드웨어 안정화 및 일정 리드 |
| 이두현 | ROS2 / Navigation | Nav2 기반 순찰/상태머신 및 주행 파라미터 튜닝 |
| 김민성 | Vision / Deploy | ArUco Auto-Docking + 카메라 파이프라인 + systemd/udev 자동 실행 |
| 류균봉 | Server / UI | Qt 관제 UI + MariaDB 적재 + MQTT 프로토콜/정합성 |

<details>
<summary><b>Details</b></summary>

### 김찬미 (STM32 Firmware & Hardware)
- **Firmware**: 센서(ADC/I2C) 수집 + 액추에이터(PWM) 제어 로직 구현
- **Hardware**: 배선/전원/GND/신호 품질 점검 및 통신 안정화
- 관련 코드: `src/SGP30_3*/`

### 이두현 (ROS2 Control & Navigation)
- **ROS2 Control**: 순찰 동작/상태 머신 구성
- **Navigation**: Nav2 파라미터 튜닝으로 주행 안정화
- 관련 코드: `src/ess_control_pkg/`, `ess_map/`

### 김민성 (Computer Vision & System Deploy)
- **Auto-Docking**: ArUco 기반 정렬/Homing 알고리즘(오차/hold/스케일/각도 보정) 구현
- **Vision Pipeline**: RGB/열화상 스트림 수집→이벤트 생성→전송 흐름 구성
- **Deploy**: systemd/udev로 부팅 자동 실행(카메라 심링크, 서비스 의존성, 재시작 정책)
- 관련 코드: `deploy/`, `src/ess_aruco_move/`, `src/...thermal...`

### 류균봉 (Central Server & Control UI)
- **Control Tower**: Qt(C++) 관제 대시보드/모니터링 UI 구현
- **Backend**: MariaDB 스키마/적재 및 로그 관리
- **Protocol**: MQTT 토픽/메시지 포맷 설계 및 정합성 관리
- 관련 코드: `src/ess_server/`

</details>
---

<div align="center">
<a href="#korean">⬆️ 한국어 버전으로 돌아가기 (Go back to Korean Version) ⬆️</a>
</div>
