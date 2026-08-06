package org.firstinspires.ftc.teamcode.part;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    
    // --- Drive Constants ---
    public static double DRIVE_SPEED_MULTIPLIER = 0.8;
    
    // --- Vision (Auto-Aim) Constants ---
    // 오토 에임 시 회전 속도를 결정하는 비례상수(P-Gain)입니다.
    // 값이 너무 작으면 타겟을 향해 도는 속도가 느리고, 너무 크면 조준점을 지나쳐 덜덜 떨립니다(발진).
    public static double VISION_TURN_KP = 0.03; 
    
    // --- Eater (Intake + Transfer) Constants ---
    public static double EATER_POWER = 1;
    public static double EATER_REVERSE_POWER = -1;
    // --- Shooter (Flywheel) Constants ---
    // 1150 RPM 모터의 경우 최대 속도가 약 2700 Ticks/sec 정도입니다.
    public static double FAR_SHOOT_VELOCITY = 2400.0;     // 멀리 쏠 때 (파슛) 속도
    public static double CLOSE_SHOOT_VELOCITY = 2100.0;   // 가까이서 쏠 때 (골밑슛) 속도
    
    // 스마트 매크로(원버튼 발사) 시간 세팅
    public static double SHOOTER_SPOOL_TIME_MS = 3000;     // 발사 전 플라이휠이 목표 속도에 도달할 때까지 기다리는 예열 시간
    public static double RAPID_FIRE_DELAY_FAR_MS = 700;   // 파슛(장거리) 연사 딜레이 (속도 회복 시간 필요)
    public static double RAPID_FIRE_DELAY_CLOSE_MS = 370;  // 골밑슛(단거리) 연사 딜레이 (빠르게 연사 가능)
    
    // --- Sensor & Feed Constants ---
    // 공이 센서에 몇 cm 이내로 접근했을 때 감지할 것인지 설정 (튜닝 필요)
    public static double EATER_BALL_DISTANCE_CM = 4.5;
    
    
    // 발사 버튼을 누를 때 트랜스퍼 모터를 몇 ms 동안 돌려서 공을 밀어넣을 것인지 설정
    public static double EATER_FEED_TIME_MS = 350;
}
