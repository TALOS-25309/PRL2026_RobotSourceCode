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
    public static Vector2d EAT_CORNER_POS = new Vector2d(60, -60);
    public static double EAT_CORNER_HEADING = Math.toRadians(-90);
    public static Pose2d EAT_CORNER_POSE = new Pose2d(EAT_CORNER_POS, EAT_CORNER_HEADING);
    public static double EAT_CORNER_TANGENT = Math.toRadians(-90);


    // TODO:실제 경기장에서 체크하기
    public static Vector2d EAT_GATE_POS = new Vector2d(15, -60);
    public static double EAT_GATE_HEADING = Math.toRadians(-145);
    public static Pose2d EAT_GATE_POSE = new Pose2d(EAT_GATE_POS, EAT_GATE_HEADING);
    public static double EAT_GATE_TANGENT = Math.toRadians(-90);

    // 이팅 대기시간
    public static double EAT_CORNER_WAIT_TIME = 1.0; //TODO
    public static double EAT_GATE_WAIT_TIME = 3.0; //TODO

}