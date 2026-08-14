package org.firstinspires.ftc.teamcode.opmode.test;

import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.*;
import static org.firstinspires.ftc.teamcode.part.Constants.*;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmode.auto.SubsystemUpdater;
import org.firstinspires.ftc.teamcode.part.eater.Eater;
import org.firstinspires.ftc.teamcode.part.shooter.Shooter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Arrays;

/**
 * 사이클 자율주행.
 *
 *   ① 발사 지점 --회전하며--> 벽 앞
 *   ② 제자리에서 회전 (벽과 나란히)
 *   ③ 벽 따라 회전 없이 이동하며 먹기
 *   ④ 인테이크 정지
 *   ⑤ 벽 --회전하며--> 발사 지점 복귀
 *   ⑥ 2발 발사
 *   위를 CYCLE_COUNT회 반복.
 *
 * [중요] 경로를 여러 개의 actionBuilder로 쪼개면, 각 builder에 넘긴 "시작 포즈"가
 * 앞 구간의 실제 끝 포즈와 어긋나면서 로봇이 엉뚱하게 움직인다.
 * 그래서 아래처럼 builder 하나를 계속 이어 붙인다. 이러면 RoadRunner가
 * 끝 포즈를 자동으로 물려받아 어긋날 여지가 없다.
 */
@Autonomous(name = "Auto Test 2", group = "Test")
public class AutoTest2 extends LinearOpMode {

    private final Eater eater = new Eater();
    private final Shooter shooter = new Shooter();



    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, FAR_START_POSE);

        eater.init(hardwareMap, telemetry);
        shooter.init(hardwareMap, telemetry);

        SubsystemUpdater updater = new SubsystemUpdater(eater, shooter);

        VelConstraint slowVel = new MinVelConstraint(Arrays.asList(
                drive.kinematics.new WheelVelConstraint(20),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint slowAccel = new ProfileAccelConstraint(-20, 20);

        // 시작 위치 --회전하며--> 발사 지점
        TrajectoryActionBuilder builder = drive.actionBuilder(FAR_START_POSE)
                .setTangent(EAT_CORNER_TANGENT)
                .strafeToLinearHeading(FAR_SHOOT_POS, FAR_SHOOT_HEADING);

        for (int i = 0; i < CYCLE_COUNT; i++) {
            builder = builder
                    // 1 발사 지점 --회전하며--> 벽 앞 (출발과 동시에 인테이크 ON)
//                    .afterTime(0.0, eater.runIntakeAction())
                    .setTangent(EAT_CORNER_TANGENT)
                    .strafeToLinearHeading(
                        EAT_CORNER_POS, EAT_CORNER_HEADING
                    )
                    .strafeToLinearHeading(
                        EAT_GATE_POS, EAT_GATE_HEADING
                    )

//                    // 2 벽 따라 회전 없이 이동하며 먹기 (heading 유지 = strafeTo, 저속)
                    .strafeTo(EAT_GATE_POS, slowVel, slowAccel)
//
//                    // 3 먹기 종료
//                    .stopAndAdd(eater.stopEaterAction())
//
//                    // 4 벽 --회전하며--> 발사 지점 복귀 (출발과 동시에 슈터 예열)
////                    .afterTime(0.0, shooter.runShooterAction(FAR_SHOOT_VELOCITY))
                    .strafeToLinearHeading(FAR_SHOOT_POS, FAR_SHOOT_HEADING);
//                    .waitSeconds(3)
//
//                    // ⑥ 2발 발사
//                    .stopAndAdd(shootTwice());
        }

        Action autoSequence = new SequentialAction(
                builder.build(),
                shooter.stopShooterAction(),
                eater.stopEaterAction(),
                new InstantAction(() -> updater.keepRunning = false)
        );

        waitForStart();
        if (isStopRequested()) return;

        eater.start();
        shooter.start();

        Actions.runBlocking(
            new ParallelAction(
                autoSequence,
                updater
            )
        );

        eater.stop();
        shooter.stop();
    }

    /** 목표 속도 도달 대기 -> 급탄 -> 속도 회복 대기 -> 급탄. */
    private Action shootTwice() {
        return new SequentialAction(
            shooter.waitUntilTargetVelocityAction(FAR_SHOOT_VELOCITY, SPINUP_TIMEOUT_S),
            eater.feedToShooterAction(),
            new SleepAction(EATER_FEED_TIME_MS / 1000.0),        // 1발째 급탄 완료 대기
            new SleepAction(RAPID_FIRE_DELAY_FAR_MS / 1000.0),   // 연사 딜레이

            shooter.waitUntilTargetVelocityAction(FAR_SHOOT_VELOCITY, SPINUP_TIMEOUT_S),
            eater.feedToShooterAction(),
            new SleepAction(EATER_FEED_TIME_MS / 1000.0)         // 2발째 급탄 완료 대기
        );
    }
}
