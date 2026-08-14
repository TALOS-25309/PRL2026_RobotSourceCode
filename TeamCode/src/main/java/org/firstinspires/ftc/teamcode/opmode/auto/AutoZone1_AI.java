package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.part.Constants;
import org.firstinspires.ftc.teamcode.part.Vision;
import org.firstinspires.ftc.teamcode.part.eater.Eater;
import org.firstinspires.ftc.teamcode.part.shooter.Shooter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Zone 1 (AI)", group = "Auto")
public class AutoZone1_AI extends LinearOpMode {

    public enum AutoState {
        SWEEPING,
        TRACKING_BALL,
        BLIND_PURSUIT,
        AIMING_GOAL,
        SHOOTING
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Eater eater = new Eater();
        Shooter shooter = new Shooter();
        Vision vision = new Vision();

        eater.init(hardwareMap, telemetry);
        shooter.init(hardwareMap, telemetry);
        vision.init(hardwareMap, telemetry);

        Vector2d[] sweepWaypoints = {
            new Vector2d(20, 0),
            new Vector2d(20, 20),
            new Vector2d(40, 20),
            new Vector2d(40, 40),
            new Vector2d(60, 40),
            new Vector2d(60, 60),
            new Vector2d(30, 60),
            new Vector2d(0, 60),
            new Vector2d(0, 30)
        };
        int waypointIndex = 0;

        AutoState state = AutoState.SWEEPING;
        double blindPursuitHeading = 0.0;
        
        ElapsedTime macroTimer = new ElapsedTime();
        int shootStep = 0;

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        eater.start();
        shooter.start();
        vision.start();
        
        vision.setPipeline(1);
        eater.runIntake(); 

        while (opModeIsActive() && !isStopRequested()) {
            drive.updatePoseEstimate();
            eater.update();
            shooter.update();
            vision.update();

            double currentX = drive.pose.position.x;
            double currentY = drive.pose.position.y;
            double currentH = drive.pose.heading.toDouble();

            switch (state) {
                case SWEEPING:
                    if (vision.hasTarget()) {
                        state = AutoState.TRACKING_BALL;
                        break;
                    }

                    Vector2d target = sweepWaypoints[waypointIndex];
                    double errorX = target.x - currentX;
                    double errorY = target.y - currentY;
                    double distance = Math.hypot(errorX, errorY);

                    if (distance < 5.0) { 
                        waypointIndex = (waypointIndex + 1) % sweepWaypoints.length;
                    }

                    double cos = Math.cos(-currentH);
                    double sin = Math.sin(-currentH);
                    double robotX = errorX * cos - errorY * sin;
                    double robotY = errorX * sin + errorY * cos;

                    double driveKp = 0.05;
                    double forwardPower = robotX * driveKp;
                    double strafePower = robotY * driveKp;
                    double turnPower = Math.sin(-currentH) * 1.0;

                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(forwardPower, strafePower), turnPower
                    ));
                    break;

                case TRACKING_BALL:
                    if (!vision.hasTarget()) {
                        blindPursuitHeading = currentH;
                        state = AutoState.BLIND_PURSUIT;
                        break;
                    }

                    double txBall = vision.getTx();
                    double trackTurn = -txBall * Constants.VISION_TURN_KP;
                    
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(0.4, 0), trackTurn
                    ));
                    break;

                case BLIND_PURSUIT:
                    if (eater.isBallDetected()) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        eater.stopEater();
                        vision.setPipeline(0);
                        state = AutoState.AIMING_GOAL;
                        break;
                    }

                    if (currentX > 70 || currentY > 70 || currentX < 0 || currentY < 0) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        state = AutoState.SWEEPING;
                        break;
                    }

                    double blindTurn = (blindPursuitHeading - currentH) * 0.5;
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(0.4, 0), blindTurn
                    ));
                    break;

                case AIMING_GOAL:
                    if (vision.hasTarget()) {
                        double txGoal = vision.getTx();
                        if (Math.abs(txGoal) < 1.5) {
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                            shooter.runShooter(Constants.FAR_SHOOT_VELOCITY);
                            macroTimer.reset();
                            shootStep = 1;
                            state = AutoState.SHOOTING;
                        } else {
                            drive.setDrivePowers(new PoseVelocity2d(
                                    new Vector2d(0, 0), -txGoal * Constants.VISION_TURN_KP
                            ));
                        }
                    } else {
                        drive.setDrivePowers(new PoseVelocity2d(
                                new Vector2d(0, 0), Math.sin(-currentH) * 1.0
                        ));
                    }
                    break;

                case SHOOTING:
                    if (shootStep == 1 && macroTimer.milliseconds() > Constants.SHOOTER_SPOOL_TIME_MS) {
                        eater.feedToShooter();
                        macroTimer.reset();
                        shootStep = 2;
                    } else if (shootStep == 2 && macroTimer.milliseconds() > Constants.EATER_FEED_TIME_MS) {
                        eater.stopEater(); 
                        macroTimer.reset();
                        shootStep = 3;
                    } else if (shootStep == 3 && macroTimer.milliseconds() > Constants.RAPID_FIRE_DELAY_FAR_MS) {
                        eater.feedToShooter();
                        macroTimer.reset();
                        shootStep = 4;
                    } else if (shootStep == 4 && macroTimer.milliseconds() > Constants.EATER_FEED_TIME_MS) {
                        shooter.stopShooter();
                        eater.stopEater();
                        
                        vision.setPipeline(1);
                        eater.runIntake();     
                        state = AutoState.SWEEPING;
                    }
                    break;
            }

            telemetry.addData("State", state.toString());
            telemetry.update();
        }
        
        eater.stop();
        shooter.stop();
        vision.stop();
    }
}
