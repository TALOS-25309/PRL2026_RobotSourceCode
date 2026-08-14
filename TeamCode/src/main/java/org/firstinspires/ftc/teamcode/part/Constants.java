package org.firstinspires.ftc.teamcode.part;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    
    // --- Drive Constants ---
    public static double DRIVE_SPEED_MULTIPLIER = 0.8;    // 수동 조종(TeleOp) 시 전체 주행 모터 속도 비율 (0.0 ~ 1.0)
    
    // --- Vision (Auto-Aim) Constants ---
    // 오토 에임 시 회전 속도를 결정하는 비례상수(P-Gain)입니다.
    // 값이 너무 작으면 타겟을 향해 도는 속도가 느리고, 너무 크면 조준점을 지나쳐 덜덜 떨립니다(발진).
    public static double VISION_TURN_KP = 0.02;          // 텔레옵 기본 사격 조준 시 (골대 에이프릴 태그) 회전 민감도
    public static double ZONE1_3_VISION_TURN_KP = 0.02;  // Zone 1, 3 골대 조준(탐색) 회전 민감도 (KP)
    public static double ZONE2_4_VISION_TURN_KP = 0.015;  // Zone 2, 4 골대 조준(탐색) 회전 민감도 (KP)
    public static double BALL_VISION_TURN_KP = 0.03;     // 오토에서 바닥에 있는 공을 쫓아갈 때의 회전 민감도

    // Zone 1 전용 사격 영점 조절 (오른쪽으로 빗나가면 양수(+), 왼쪽으로 빗나가면 음수(-) 입력)
    public static double ZONE1_SHOOT_TX_OFFSET = 0; // 사격 시 좌우 오차 보정을 위한 카메라 오프셋 각도
    public static double ZONE2_SHOOT_TX_OFFSET = 0; // Zone 2 사격 시 카메라 오프셋
    public static double ZONE3_SHOOT_TX_OFFSET = 0; // Zone 3 사격 시 카메라 오프셋
    public static double ZONE4_SHOOT_TX_OFFSET = 0; // Zone 4 사격 시 카메라 오프셋
    
    // 각 자율주행 구역(Zone)별 타겟 에이프릴 태그 ID (라임라이트가 해당 ID만 추적)
    public static int ZONE1_TARGET_TAG_ID = 20; // Zone 1 목표 골대 태그 ID
    public static int ZONE2_TARGET_TAG_ID = 20; // Zone 2 목표 골대 태그 ID
    public static int ZONE3_TARGET_TAG_ID = 24; // Zone 3 목표 골대 태그 ID
    public static int ZONE4_TARGET_TAG_ID = 24; // Zone 4 목표 골대 태그 ID
    
    // 각 자율주행 구역(Zone)별 기본 사격 목표 각도 (도, Degrees)
    public static double ZONE1_SHOOT_HEADING_DEG = 20.0;  // Zone 1 사격 기준 각도
    public static double ZONE2_SHOOT_HEADING_DEG = 45.0;  // Zone 2 사격 기준 각도
    public static double ZONE3_SHOOT_HEADING_DEG = -20.0; // Zone 3 사격 기준 각도
    public static double ZONE4_SHOOT_HEADING_DEG = 0.0;   // Zone 4 사격 기준 각도 (추후 설정)
    
    // 각 자율주행 구역(Zone)별 시작 전진 거리 및 사격 위치 좌표 (인치)
    public static double ZONE1_INITIAL_FORWARD_INCHES = 15.0; // 시작 시 벽에서 떨어져 탐색을 시작할 전진 거리
    public static double ZONE1_SHOOT_POSE_X = 12.0;           // Zone 1 사격 위치 X 좌표
    public static double ZONE1_SHOOT_POSE_Y = 0.0;            // Zone 1 사격 위치 Y 좌표
    
    public static double ZONE2_INITIAL_FORWARD_INCHES = 15.0; // Zone 2 탐색 시작 전진 거리
    public static double ZONE2_SHOOT_POSE_X = 0.0;            // Zone 2 사격 위치 X 좌표
    public static double ZONE2_SHOOT_POSE_Y = 0.0;            // Zone 2 사격 위치 Y 좌표
    
    public static double ZONE3_INITIAL_FORWARD_INCHES = 15.0; // Zone 3 탐색 시작 전진 거리
    public static double ZONE3_SHOOT_POSE_X = 12.0;           // Zone 3 사격 위치 X 좌표
    public static double ZONE3_SHOOT_POSE_Y = 0.0;            // Zone 3 사격 위치 Y 좌표
    
    public static double ZONE4_INITIAL_FORWARD_INCHES = 15.0; // Zone 4 탐색 시작 전진 거리
    public static double ZONE4_SHOOT_POSE_X = 15.0;           // Zone 4 사격 위치 X 좌표
    public static double ZONE4_SHOOT_POSE_Y = 0.0;            // Zone 4 사격 위치 Y 좌표
    
    // 각 자율주행 구역(Zone)별 시작/사격 후 복귀할 탐색 시작 위치 (오른쪽 앞 모서리 명당)
    public static double ZONE1_SEARCH_START_X = 50.0;         // Zone 1 탐색 시작 X 좌표
    public static double ZONE1_SEARCH_START_Y = 0.0;         // Zone 1 탐색 시작 Y 좌표 (-값이 오른쪽)
    public static double ZONE1_SEARCH_START_HEADING = 90.0;   // Zone 1 탐색 시작 시 바라볼 각도 (도)
    
    public static double ZONE2_SEARCH_START_X = 0.0;
    public static double ZONE2_SEARCH_START_Y = 0.0;
    public static double ZONE2_SEARCH_START_HEADING = 0.0;
    
    public static double ZONE3_SEARCH_START_X = 50.0;
    public static double ZONE3_SEARCH_START_Y = 0.0;
    public static double ZONE3_SEARCH_START_HEADING = -180.0;
    
    public static double ZONE4_SEARCH_START_X = 45.0;
    public static double ZONE4_SEARCH_START_Y = -5.0;
    public static double ZONE4_SEARCH_START_HEADING = 90.0;
    
    // --- Eater (Intake + Transfer) Constants ---
    public static double EATER_POWER = 1;                 // 공 흡입(Intake) 및 트랜스퍼 모터 정방향 작동 파워
    public static double EATER_REVERSE_POWER = -1;        // 공 뱉어내기(Reverse) 모터 역방향 작동 파워
    
    // --- Shooter (Flywheel) Constants ---
    // 1150 RPM 모터의 경우 최대 속도가 약 2700 Ticks/sec 정도입니다.
    public static double FAR_SHOOT_VELOCITY = 2310.0;     // 멀리 쏠 때 (파슛) 플라이휠 목표 속도 (Ticks/sec)
    public static double CLOSE_SHOOT_VELOCITY = 2100.0;   // 가까이서 쏠 때 (골밑슛) 플라이휠 목표 속도 (Ticks/sec)

    public static double VEL_TOLERANCE = 50;              // 플라이휠 조준 완료로 인정하는 목표 속도와의 허용 오차 범위 (Ticks/sec)
    
    // 스마트 매크로(원버튼 발사) 시간 세팅
    public static double SHOOTER_SPOOL_TIME_MS = 3000;     // 발사 전 플라이휠이 목표 속도에 도달할 때까지 기다리는 예열 시간
    public static double RAPID_FIRE_DELAY_FAR_MS = 700;   // 파슛(장거리) 연사 딜레이 (속도 회복 시간 필요)
    public static double RAPID_FIRE_DELAY_CLOSE_MS = 370;  // 골밑슛(단거리) 연사 딜레이 (빠르게 연사 가능)
    
    // --- Sensor & Feed Constants ---
    // 공이 센서에 몇 cm 이내로 접근했을 때 감지할 것인지 설정 (튜닝 필요)
    public static double EATER_BALL_DISTANCE_CM = 4.5;    // 인테이크 흡입 감지 거리 임계값 (cm)
    
    // 발사 버튼을 누를 때 트랜스퍼 모터를 몇 ms 동안 돌려서 공을 밀어넣을 것인지 설정
    public static double EATER_FEED_TIME_MS = 350;        // 슈터로 공을 공급하는 모터 구동 시간 (ms)

    // --- Auto (AI) Constants ---
    public static double AUTO_SPIN_SPEED = 0.3;           // 제자리 스캔 회전 속도 (공 탐색 전용)
    public static double AUTO_AIM_MIN_SPEED = 0.25;      // 사격 조준 시 마찰력을 이기기 위한 최소 회전 파워 (기존 0.3에서 모터 멈춤/고주파음 현상 방지를 위해 0.45로 상향)
    public static double AUTO_ESCAPE_SPIN_SPEED = 0.5;    // 구역 경계에 닿았을 때 튕겨나오며 도는 속도
    public static double ZONE_LENGTH_X = 53.93701;        // 각 Zone의 세로(X축) 크기 (인치)
    public static double ZONE_WIDTH_Y = 55.90551;         // 각 Zone의 가로(Y축) 크기 (인치)
    public static double AUTO_WALL_EXTENSION_INCHES = 1.5;// 벽이 있는 곳의 여유분 (물리적 벽에 닿기 전에 포기하는 것을 막기 위해 구역을 벽 쪽으로 넓히는 여유분)
    public static double AUTO_OPEN_EXTENSION_INCHES = 3.0;// 벽이 없는 열린 곳의 여유분 (공을 쫓아갈 때 구역 밖으로 나가는 것을 허용하는 여유분)
    public static double AUTO_DRIVE_KP = 0.05;             // 지정 위치(명당/웨이포인트)로 이동 시 속도 계수 (P-Gain)
    public static double AUTO_MAX_DRIVE_SPEED = 0.52;     // 웨이포인트/사격장 이동 시 최대 속도 제한 (미끄러짐 방지)
    public static double AUTO_FORWARD_SPEED = 0.3;         // 공을 쫓아갈 때 전진 속도
    public static double AUTO_PURSUIT_TIMEOUT_MS = 4500;   // 공 추적(사각지대 포함) 시 획득 실패로 간주하는 시간 (ms)
    public static double AUTO_ABORT_REVERSE_TIME_MS = 500; // 획득 실패 시 뒤로 후진하는 시간 (ms)
    public static double AUTO_JAM_CURRENT_LIMIT = 7.0;    // 슈터 과전류/걸림 판단 임계값 (A)
    public static double AUTO_JAM_REVERSE_TIME_MS = 300.0;// 슈터 걸림 시 트랜스퍼 역회전 작동 시간 (ms)
}
