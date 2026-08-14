package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.*;
import static org.firstinspires.ftc.teamcode.part.Constants.*;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.part.eater.Eater;
import org.firstinspires.ftc.teamcode.part.shooter.Shooter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name ="CloseZone+Gate", group = "Auto")
public class AutoOp_CloseGate_old extends LinearOpMode {

    private final Eater eater = new Eater();
    private final Shooter shooter = new Shooter();

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, CLOSE_START_POSE);

        eater.init(hardwareMap, telemetry);
        shooter.init(hardwareMap, telemetry);

        SubsystemUpdater updater = new SubsystemUpdater(eater, shooter);


        TrajectoryActionBuilder builder = drive.actionBuilder(CLOSE_START_POSE)
                .strafeToLinearHeading(CLOSE_SHOOT_POS, CLOSE_SHOOT_HEADING)
                .stopAndAdd(shootTwice());

        for (int i = 0; i < CYCLE_COUNT; i++) {
            builder = builder
                    .afterTime(0.0, eater.runIntakeAction())
                    .setTangent(EAT_GATE_TANGENT)
                    .strafeToLinearHeading(EAT_GATE_POS, EAT_GATE_HEADING)

                    .afterTime(EAT_GATE_WAIT_TIME, eater.stopEaterAction())

                    .afterTime(0.0, shooter.runShooterAction(CLOSE_SHOOT_VELOCITY))
                    .strafeToLinearHeading(CLOSE_SHOOT_POS, CLOSE_SHOOT_HEADING)
                    .stopAndAdd(shootTwice());
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
                shooter.waitUntilTargetVelocityAction(CLOSE_SHOOT_VELOCITY, SPINUP_TIMEOUT_S),
                eater.feedToShooterAction(),
                new SleepAction(EATER_FEED_TIME_MS / 1000.0),        // 1발째 급탄 완료 대기
                new SleepAction(RAPID_FIRE_DELAY_CLOSE_MS / 1000.0),   // 연사 딜레이

                shooter.waitUntilTargetVelocityAction(CLOSE_SHOOT_VELOCITY, SPINUP_TIMEOUT_S),
                eater.feedToShooterAction(),
                new SleepAction(EATER_FEED_TIME_MS / 1000.0)         // 2발째 급탄 완료 대기
        );
    }
}
