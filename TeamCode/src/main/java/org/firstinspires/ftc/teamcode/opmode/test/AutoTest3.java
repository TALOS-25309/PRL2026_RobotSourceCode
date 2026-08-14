package org.firstinspires.ftc.teamcode.opmode.test;

import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.CYCLE_COUNT;
import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.EAT_CORNER_HEADING;
import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.EAT_CORNER_POS;
import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.FAR_SHOOT_HEADING;
import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.FAR_SHOOT_POS;
import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.SPINUP_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.FAR_START_POSE;
import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.EAT_CORNER_TANGENT;
import static org.firstinspires.ftc.teamcode.part.Constants.EATER_FEED_TIME_MS;
import static org.firstinspires.ftc.teamcode.part.Constants.FAR_SHOOT_VELOCITY;
import static org.firstinspires.ftc.teamcode.part.Constants.RAPID_FIRE_DELAY_FAR_MS;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmode.auto.SubsystemUpdater;
import org.firstinspires.ftc.teamcode.part.eater.Eater;
import org.firstinspires.ftc.teamcode.part.shooter.Shooter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Auto Test 3", group = "Test")
public class AutoTest3 extends LinearOpMode {

    private final Eater eater = new Eater();
    private final Shooter shooter = new Shooter();



    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, FAR_START_POSE);

        eater.init(hardwareMap, telemetry);
        shooter.init(hardwareMap, telemetry);

        SubsystemUpdater updater = new SubsystemUpdater(eater, shooter);


        // 시작 위치 --회전하며--> 발사 지점
        TrajectoryActionBuilder builder = drive.actionBuilder(FAR_START_POSE)
                .setTangent(EAT_CORNER_TANGENT)
                .strafeToLinearHeading(FAR_SHOOT_POS, FAR_SHOOT_HEADING)
                .stopAndAdd(shootTwice());

        for (int i = 0; i < CYCLE_COUNT; i++) {
            builder = builder
                    // 1. 발사 지점 --회전하며--> 벽 앞 (출발과 동시에 인테이크 ON)
//                    .afterTime(0.0, eater.runIntakeAction())
//                    .setTangent(WALL_ENTRY_TANGENT)
                    .strafeToLinearHeading(
                        EAT_CORNER_POS, EAT_CORNER_HEADING
                    )

                    // 2 먹기 종료
//                    .stopAndAdd(eater.stopEaterAction())

                    // 3 벽 --회전하며--> 발사 지점 복귀 (출발과 동시에 슈터 예열)
//                    .afterTime(0.0, shooter.runShooterAction(FAR_SHOOT_VELOCITY))
                    .strafeToLinearHeading(FAR_SHOOT_POS, FAR_SHOOT_HEADING);

                    // ⑥ 2발 발사
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
