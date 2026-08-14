package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * 자율주행 좌표 상수.
 *
 * [좌표계]
 *   단위 = 인치(1in = 2.54cm), 각도 = 라디안, 반시계(CCW)가 +
 *   원점 = 필드 중앙 (로봇 시작 위치가 아니라 필드 중앙 고정)
 *   +x  = 시뮬레이터 화면상 위쪽 (far wall 방향)
 *   +y  = 시뮬레이터 화면상 왼쪽
 *
 */
@Config("AUTO")
public class Constants {

    // 사이클 반복 횟수
    public static int CYCLE_COUNT = 1;

    // 플라이휠이 목표 속도에 도달할 때까지 기다리는 최대 시간(초)
    public static double SPINUP_TIMEOUT_S = 2.0;

    // --- 발사 전 비전 조준 (Limelight 파이프라인 0 = Goal) ---
    public static int VISION_GOAL_PIPELINE = 0;
    public static double AIM_TOLERANCE_DEG = 1.5;   // tx 가 이 안에 들어오면 정렬 완료
    public static double AIM_TIMEOUT_S = 2.0;       // 조준 최대 시간(초). 초과하면 그냥 발사
    public static double AIM_MAX_TURN = 0.3;        // 조준 회전 파워 상한 (화면 끝에서 폭주 방지)
    public static double AIM_MIN_TURN = 0.08;       // 정지 마찰을 이기기 위한 최소 회전 파워
    public static double AIM_SEARCH_TURN = 0.2;     // 골대가 안 보일 때 제자리 탐색 회전 파워

    // 인테이크 구간 주행 속도 제한 (in/s). 기본값은 50이라 공이 튕겨나간다.
    public static double EAT_MAX_VEL = 20.0;
    public static double EAT_MAX_ACCEL = 20.0;


    // 시작 지점
    public static Pose2d FAR_START_POSE = new Pose2d(new Vector2d(60, -10), Math.toRadians(180));
    public static Pose2d CLOSE_START_POSE = new Pose2d(new Vector2d(0, 0), Math.toRadians(0)); //TODO


    // 발사 지점
    public static Vector2d FAR_SHOOT_POS = new Vector2d(50, -20);
    public static double FAR_SHOOT_HEADING = Math.toRadians(-160);
    public static Pose2d FAR_SHOOT_POSE = new Pose2d(FAR_SHOOT_POS, FAR_SHOOT_HEADING);

    public static Vector2d CLOSE_SHOOT_POS = new Vector2d(-22, -17);
    public static double CLOSE_SHOOT_HEADING = Math.toRadians(-140);
    public static Pose2d CLOSE_SHOOT_POSE = new Pose2d(CLOSE_SHOOT_POS, CLOSE_SHOOT_HEADING);


    // 이팅 지점
    public static Vector2d EAT_CORNER_POS = new Vector2d(60, -65);
    public static double EAT_CORNER_HEADING = Math.toRadians(-90);
    public static Pose2d EAT_CORNER_POSE = new Pose2d(EAT_CORNER_POS, EAT_CORNER_HEADING);
    public static double EAT_CORNER_TANGENT = Math.toRadians(-90);


    // TODO:실제 경기장에서 체크하기
    public static Vector2d EAT_GATE_POS = new Vector2d(15, -60);
    public static double EAT_GATE_HEADING = Math.toRadians(-145);
    public static Pose2d EAT_GATE_POSE = new Pose2d(EAT_GATE_POS, EAT_GATE_HEADING);
    public static double EAT_GATE_TANGENT = Math.toRadians(-90);

    // 이팅 대기시간
    public static double EAT_CORNER_WAIT_TIME = 2.0;
    public static double EAT_GATE_WAIT_TIME = 3.0; //TODO



    public static double DELAY_BETWEEN_TWO_INTAKE = 0.7; //TO
}