package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

/**
 * AutoTest2와 같은 경로를 PC에서 미리 그려보는 시뮬레이터.
 *
 * AXIS_CHECK = true 로 두면 경로 대신 "축 확인용" 주행을 그린다.
 *   (0,0) -> (48,0) : 화면에서 +x가 어느 쪽인지 확인
 *   (48,0) -> (48,48) : 화면에서 +y가 어느 쪽인지 확인
 * 이걸 먼저 돌려서 화면상 축 방향을 눈으로 확인한 뒤 좌표를 정하면
 * 부호 때문에 반대로 가는 실수를 안 한다.
 */
public class MeepMeepTesting {

    private static final boolean AXIS_CHECK = true;

    // ── AutoTest2의 opmode/auto/Constants.java 와 같은 값으로 맞춰둘 것 ──
    private static final Pose2d START_POSE = new Pose2d(new Vector2d(36, 36), Math.toRadians(0));

    private static final Vector2d SHOOT_POS = new Vector2d(36, 24);
    private static final double SHOOT_HEADING = Math.toRadians(-45);

    private static final Vector2d WALL_ENTRY_POS = new Vector2d(44, -47);
    private static final double WALL_ENTRY_HEADING = Math.toRadians(-90);

    private static final double WALL_SWEEP_HEADING = Math.toRadians(180);

    private static final Vector2d WALL_END_POS = new Vector2d(-20, -47);

    private static final int CYCLE_COUNT = 1;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17, 17)   // 로봇 크기(인치)
                .build();

        bot.runAction(AXIS_CHECK ? axisCheck(bot) : autoPath(bot));

        meepMeep.setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // 필드 배경을 넣고 싶으면 아래 주석을 풀고 MeepMeep.Background 자동완성에서 고를 것
                // .setBackground(MeepMeep.Background.GRID_GRAY)
                .addEntity(bot)
                .start();
    }

    /** 화면에서 +x / +y가 각각 어느 방향인지 확인용. */
    private static com.acmerobotics.roadrunner.Action axisCheck(RoadRunnerBotEntity bot) {
        return bot.getDrive().actionBuilder(new Pose2d(new Vector2d(0, 0), 0))
                .strafeTo(new Vector2d(48, 0))    // +x 방향
                .waitSeconds(1)
                .strafeTo(new Vector2d(48, 48))   // +y 방향
                .waitSeconds(1)
                .build();
    }

    /** AutoTest2와 동일한 사이클 경로. */
    private static com.acmerobotics.roadrunner.Action autoPath(RoadRunnerBotEntity bot) {
        var builder = bot.getDrive().actionBuilder(START_POSE)
                .strafeToLinearHeading(SHOOT_POS, SHOOT_HEADING)
                .waitSeconds(0.5);

        for (int i = 0; i < CYCLE_COUNT; i++) {
            builder = builder
                    // ① 발사 지점 --회전하며--> 벽 앞
                    .strafeToLinearHeading(WALL_ENTRY_POS, WALL_ENTRY_HEADING)
                    .waitSeconds(0.5)
                    // ② 제자리에서 회전
                    .turnTo(WALL_SWEEP_HEADING)
                    .waitSeconds(0.5)
                    // ③ 벽 따라 회전 없이 이동
                    .strafeTo(WALL_END_POS)
                    .waitSeconds(0.5)
                    // ⑤ 발사 지점 복귀
                    .strafeToLinearHeading(SHOOT_POS, SHOOT_HEADING)
                    .waitSeconds(0.5);
        }

        return builder.build();
    }
}
