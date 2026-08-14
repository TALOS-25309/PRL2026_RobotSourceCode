package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.CYCLE_COUNT;
import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.EAT_CORNER_HEADING;
import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.EAT_CORNER_POS;
import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.EAT_CORNER_TANGENT;
import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.EAT_CORNER_WAIT_TIME;
import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.EAT_GATE_HEADING;
import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.EAT_GATE_POS;
import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.FAR_SHOOT_HEADING;
import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.FAR_SHOOT_POS;
import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.FAR_START_POSE;
import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.SPINUP_TIMEOUT_S;
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

import org.firstinspires.ftc.teamcode.feature.CancelableAction;
import org.firstinspires.ftc.teamcode.feature.DeferredAction;
import org.firstinspires.ftc.teamcode.part.eater.Eater;
import org.firstinspires.ftc.teamcode.part.shooter.Shooter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Collections;
import java.util.List;

@Autonomous(name ="FarZone+Sweep", group = "Auto")
public class AutoOp_FarSweep extends LinearOpMode {

    private final Eater eater = new Eater();
    private final Shooter shooter = new Shooter();

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, FAR_START_POSE);

        eater.init(hardwareMap, telemetry);
        shooter.init(hardwareMap, telemetry);

        SubsystemUpdater updater = new SubsystemUpdater(eater, shooter);


        TrajectoryActionBuilder builder = drive.actionBuilder(FAR_START_POSE)
            .strafeToLinearHeading(FAR_SHOOT_POS, FAR_SHOOT_HEADING)
            .stopAndAdd(shootTwice());

        List<SequentialAction> cycles = Collections.emptyList();

        for (int i = 0; i < CYCLE_COUNT; i++) {
//            builder = builder
//                    .afterTime(0.0, eater.runIntakeAction())
//                    .setTangent(EAT_CORNER_TANGENT)
//                    .strafeToLinearHeading(EAT_CORNER_POS, EAT_CORNER_HEADING)
//                    .strafeToLinearHeading(EAT_CORNER_POS, EAT_GATE_HEADING)
//                    .strafeToLinearHeading(EAT_GATE_POS, EAT_GATE_HEADING)
//
//                    .afterTime(0.0, eater.stopEaterAction())
//
//                    .afterTime(0.0, shooter.runShooterAction(FAR_SHOOT_VELOCITY))
//                    .strafeToLinearHeading(FAR_SHOOT_POS, FAR_SHOOT_HEADING)
//                    .stopAndAdd(shootTwice());

            cycles.add(new SequentialAction(
                    eater.runIntakeAction(),

                    // ↓ 이 3줄짜리 경로가 통째로 취소 대상
                    new CancelableAction(drive, eater::isFull, new DeferredAction(() ->
                            drive.actionBuilder(drive.localizer.getPose())
                                    .setTangent(EAT_CORNER_TANGENT)
                                    .strafeToLinearHeading(EAT_CORNER_POS, EAT_CORNER_HEADING)
                                    .strafeToLinearHeading(EAT_CORNER_POS, EAT_GATE_HEADING)
                                    .strafeToLinearHeading(EAT_GATE_POS, EAT_GATE_HEADING)
                                    .build())),

                    eater.stopEaterAction(),          // ← 취소되면 곧바로 여기로 넘어옴
                    new SleepAction(0.15),            // 잔여 관성 정리 (아래 주의사항 참고)

                    shooter.runShooterAction(FAR_SHOOT_VELOCITY),
                    new DeferredAction(() ->
                            drive.actionBuilder(drive.localizer.getPose())
                                    .strafeToLinearHeading(FAR_SHOOT_POS, FAR_SHOOT_HEADING)
                                    .build()),
                    shootTwice()
            ));
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
