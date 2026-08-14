package org.firstinspires.ftc.teamcode.opmode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.part.Constants;
import org.firstinspires.ftc.teamcode.part.Vision;
import org.firstinspires.ftc.teamcode.part.eater.Eater;
import org.firstinspires.ftc.teamcode.part.shooter.Shooter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Zone 1 (Sweep & Shoot)", group = "Auto")
public class AutoZone1 extends LinearOpMode {

    public class AutoAimAction implements Action {
        private final MecanumDrive drive;
        private final Vision vision;

        public AutoAimAction(MecanumDrive drive, Vision vision) {
            this.drive = drive;
            this.vision = vision;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            vision.update();
            if (vision.hasTarget()) {
                double tx = vision.getTx();
                if (Math.abs(tx) < 1.5) {
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    return false;
                }

                double turnPower = -tx * Constants.VISION_TURN_KP;
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), turnPower));
                drive.updatePoseEstimate();
                return true;
            } else {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                return false;
            }
        }
    }

    @Override
    public void runOpMode() {
        Pose2d currentPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, currentPose);
        Eater eater = new Eater();
        Shooter shooter = new Shooter();
        Vision vision = new Vision();

        eater.init(hardwareMap, telemetry);
        shooter.init(hardwareMap, telemetry);
        vision.init(hardwareMap, telemetry);

        SubsystemUpdater updater = new SubsystemUpdater(eater, shooter, vision);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        eater.start();
        shooter.start();
        vision.start();

        Action shootTwoBalls = new SequentialAction(
                shooter.runShooterAction(Constants.FAR_SHOOT_VELOCITY),
                new SleepAction(Constants.SHOOTER_SPOOL_TIME_MS / 1000.0),
                eater.feedToShooterAction(),
                new SleepAction(Constants.EATER_FEED_TIME_MS / 1000.0),
                new SleepAction(Constants.RAPID_FIRE_DELAY_FAR_MS / 1000.0),
                eater.feedToShooterAction(),
                new SleepAction(Constants.EATER_FEED_TIME_MS / 1000.0),
                shooter.stopShooterAction()
        );

        Action sweepAndShootSequence = new SequentialAction(
                eater.runIntakeAction(),
                drive.actionBuilder(currentPose).lineToX(30).turnTo(0).build(),
                eater.stopEaterAction(),
                new AutoAimAction(drive, vision),
                shootTwoBalls,

                eater.runIntakeAction(),
                drive.actionBuilder(new Pose2d(30, 0, 0)).lineToY(30).turnTo(0).build(),
                eater.stopEaterAction(),
                new AutoAimAction(drive, vision),
                shootTwoBalls,

                eater.runIntakeAction(),
                drive.actionBuilder(new Pose2d(30, 30, 0)).lineToX(60).turnTo(0).build(),
                eater.stopEaterAction(),
                new AutoAimAction(drive, vision),
                shootTwoBalls,

                eater.runIntakeAction(),
                drive.actionBuilder(new Pose2d(60, 30, 0)).lineToY(60).turnTo(0).build(),
                eater.stopEaterAction(),
                new AutoAimAction(drive, vision),
                shootTwoBalls,

                eater.runIntakeAction(),
                drive.actionBuilder(new Pose2d(60, 60, 0)).lineToX(30).turnTo(0).build(),
                eater.stopEaterAction(),
                new AutoAimAction(drive, vision),
                shootTwoBalls,
                
                drive.actionBuilder(new Pose2d(30, 60, 0)).lineToY(0).lineToX(0).turnTo(0).build(),
                new InstantAction(() -> updater.keepRunning = false)
        );

        Actions.runBlocking(
                new ParallelAction(
                        sweepAndShootSequence,
                        updater
                )
        );

        eater.stop();
        shooter.stop();
        vision.stop();
    }
}
