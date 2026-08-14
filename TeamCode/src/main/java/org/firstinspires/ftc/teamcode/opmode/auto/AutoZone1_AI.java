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

@Autonomous(name = "Zone 1 (Auto)", group = "Auto")
public class AutoZone1_AI extends LinearOpMode {

    public enum AutoState {
        INITIAL_FORWARD, // 시작 시 벽에서 떨어지기 위해 전진
        SEARCH_SPIN,    
        SEARCH_MOVE,    
        TRACKING_BALL,  
        BLIND_PURSUIT,  
        ABORT_REVERSE,  
        ABORT_RECOVERY_SPIN, // 포기 후 방금 본 공을 시야에서 지우기 위해 강제 회전
        MOVE_TO_SHOOT,  
        AIMING_GOAL,    
        SHOOTING,       
        PIPELINE_WAIT   
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

        Vector2d[] safeWaypoints = {
            new Vector2d(30, 30),
            new Vector2d(30, 0),
            new Vector2d(60, 30),
            new Vector2d(60, 60),
            new Vector2d(30, 60),
            new Vector2d(10, 30)
        };
        int waypointIndex = 0;

        AutoState state = AutoState.INITIAL_FORWARD;
        double searchStartH = 0.0;
        double blindPursuitHeading = 0.0;
        
        ElapsedTime macroTimer = new ElapsedTime();
        ElapsedTime abortTimer = new ElapsedTime(); 
        int shootStep = 0;

        telemetry.addLine("Auto Zone 1 Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        eater.start();
        shooter.start();
        vision.start();
        
        if (eater.isBallDetected()) {
            state = AutoState.MOVE_TO_SHOOT;
            eater.stopEater();
        } else {
            vision.setPipeline(1);
            eater.runIntake(); 
        }

        while (opModeIsActive() && !isStopRequested()) {
            drive.updatePoseEstimate();
            eater.update();
            shooter.update();
            vision.update();

            double currentX = drive.localizer.getPose().position.x;
            double currentY = drive.localizer.getPose().position.y;
            double currentH = drive.localizer.getPose().heading.toDouble();

            switch (state) {
                case INITIAL_FORWARD:
                    // 시작 지점(벽면)에서 회전하면 로봇이 벽에 쓸리므로 15인치 앞으로 나온 뒤 스캔 시작
                    if (vision.hasTarget()) {
                        abortTimer.reset();
                        state = AutoState.TRACKING_BALL;
                        break;
                    }
                    
                    if (currentX < 15.0) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(Constants.AUTO_FORWARD_SPEED, 0), 0));
                    } else {
                        searchStartH = currentH;
                        state = AutoState.SEARCH_SPIN;
                    }
                    break;

                case SEARCH_SPIN:
                    if (vision.hasTarget()) {
                        abortTimer.reset(); 
                        state = AutoState.TRACKING_BALL;
                        break;
                    }

                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), Constants.AUTO_SPIN_SPEED));
                    
                    if (Math.abs(currentH - searchStartH) >= 2 * Math.PI) {
                        state = AutoState.SEARCH_MOVE;
                        macroTimer.reset(); 
                    }
                    break;

                case SEARCH_MOVE:
                    if (vision.hasTarget()) {
                        abortTimer.reset(); 
                        state = AutoState.TRACKING_BALL;
                        break;
                    }

                    Vector2d searchTarget = safeWaypoints[waypointIndex];
                    double errorX = searchTarget.x - currentX;
                    double errorY = searchTarget.y - currentY;
                    double distance = Math.hypot(errorX, errorY);

                    if (distance < 5.0 || macroTimer.milliseconds() > 1500) { 
                        waypointIndex = (waypointIndex + 1) % safeWaypoints.length;
                        searchStartH = currentH; 
                        state = AutoState.SEARCH_SPIN;
                        break;
                    }

                    double cos = Math.cos(-currentH);
                    double sin = Math.sin(-currentH);
                    double robotX = errorX * cos - errorY * sin;
                    double robotY = errorX * sin + errorY * cos;

                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(robotX * Constants.AUTO_DRIVE_KP, robotY * Constants.AUTO_DRIVE_KP), 0
                    ));
                    break;

                case TRACKING_BALL:
                    if (eater.isBallDetected()) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); 
                        eater.stopEater(); 
                        state = AutoState.MOVE_TO_SHOOT;
                        break;
                    }

                    if (abortTimer.milliseconds() > Constants.AUTO_PURSUIT_TIMEOUT_MS) {
                        state = AutoState.ABORT_REVERSE;
                        macroTimer.reset();
                        break;
                    }
                    
                    if (currentX > 75 || currentY > 75 || currentX < -5 || currentY < -5) {
                        state = AutoState.ABORT_REVERSE;
                        macroTimer.reset();
                        break;
                    }

                    if (!vision.hasTarget()) {
                        blindPursuitHeading = currentH;
                        state = AutoState.BLIND_PURSUIT;
                        break;
                    }

                    double txBall = vision.getTx();
                    double trackTurn = -txBall * Constants.VISION_TURN_KP;
                    // 공을 쫓아갈 때 회전(조향)이 너무 급격하지 않도록 최대 속도를 0.2로 제한
                    if (trackTurn > 0.2) trackTurn = 0.2;
                    if (trackTurn < -0.2) trackTurn = -0.2;
                    
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(Constants.AUTO_FORWARD_SPEED, 0), trackTurn
                    ));
                    break;

                case BLIND_PURSUIT:
                    if (eater.isBallDetected()) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); 
                        eater.stopEater(); 
                        state = AutoState.MOVE_TO_SHOOT;
                        break;
                    }
                    
                    if (vision.hasTarget()) {
                        state = AutoState.TRACKING_BALL;
                        break;
                    }

                    if (abortTimer.milliseconds() > Constants.AUTO_PURSUIT_TIMEOUT_MS) {
                        state = AutoState.ABORT_REVERSE;
                        macroTimer.reset();
                        break;
                    }

                    if (currentX > 75 || currentY > 75 || currentX < -5 || currentY < -5) {
                        state = AutoState.ABORT_REVERSE; 
                        macroTimer.reset();
                        break;
                    }

                    double blindTurn = (blindPursuitHeading - currentH) * 0.5;
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(Constants.AUTO_FORWARD_SPEED, 0), blindTurn
                    ));
                    break;

                case ABORT_REVERSE:
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-Constants.AUTO_FORWARD_SPEED, 0), 0));
                    
                    if (macroTimer.milliseconds() > Constants.AUTO_ABORT_REVERSE_TIME_MS) {
                        macroTimer.reset(); 
                        state = AutoState.ABORT_RECOVERY_SPIN;
                    }
                    break;

                case ABORT_RECOVERY_SPIN:
                    // 구역 밖의 공을 쫓다가 후진한 경우, 다시 그 공을 쫓는 무한 루프를 방지하기 위해
                    // 1.5초 동안 공을 무시하고 시야를 다른 곳으로 강제 회전시킴
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), Constants.AUTO_SPIN_SPEED));
                    
                    if (macroTimer.milliseconds() > 1500) {
                        searchStartH = currentH;
                        state = AutoState.SEARCH_SPIN;
                    }
                    break;

                case MOVE_TO_SHOOT:
                    // 사격 위치를 시작 위치(0,0)에서 앞으로 조금(15인치) 전진한 곳으로 설정
                    double shootTargetX = 15.0;
                    double shootTargetY = 0.0;
                    
                    double moveErrorX = shootTargetX - currentX;
                    double moveErrorY = shootTargetY - currentY;
                    double moveDistance = Math.hypot(moveErrorX, moveErrorY);

                    if (moveDistance < 5.0) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        vision.setPipeline(0); 
                        state = AutoState.AIMING_GOAL;
                        break;
                    }

                    double c = Math.cos(-currentH);
                    double s = Math.sin(-currentH);
                    double robX = moveErrorX * c - moveErrorY * s;
                    double robY = moveErrorX * s + moveErrorY * c;

                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(robX * Constants.AUTO_DRIVE_KP, robY * Constants.AUTO_DRIVE_KP), 0
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
                            double aimTurn = -txGoal * Constants.VISION_TURN_KP;
                            
                            // 화면 가장자리에서 발견했을 때 파워가 폭주하여 급가속하는 현상 방지 (최대 0.3으로 제한)
                            if (aimTurn > 0.3) aimTurn = 0.3;
                            if (aimTurn < -0.3) aimTurn = -0.3;

                            // 마찰력을 이기고 움직이기 위한 최소 파워(Feedforward) 설정
                            if (Math.abs(aimTurn) < Constants.AUTO_SPIN_SPEED) {
                                aimTurn = Math.signum(aimTurn) * Constants.AUTO_SPIN_SPEED;
                            }
                            drive.setDrivePowers(new PoseVelocity2d(
                                    new Vector2d(0, 0), aimTurn
                            ));
                        }
                    } else {
                        drive.setDrivePowers(new PoseVelocity2d(
                                new Vector2d(0, 0), Constants.AUTO_SPIN_SPEED
                        ));
                    }
                    break;

                case SHOOTING:
                    if (shootStep == 1 && macroTimer.milliseconds() > Constants.SHOOTER_SPOOL_TIME_MS) {
                        eater.feedToShooter();
                        macroTimer.reset();
                        shootStep = 2;
                    } else if (shootStep == 2 && macroTimer.milliseconds() > Constants.EATER_FEED_TIME_MS) {
                        shooter.stopShooter();
                        eater.stopEater();
                        
                        vision.setPipeline(1);
                        eater.runIntake();
                        
                        macroTimer.reset();
                        state = AutoState.PIPELINE_WAIT;
                    }
                    break;

                case PIPELINE_WAIT:
                    if (macroTimer.milliseconds() > 200) {
                        searchStartH = currentH; 
                        state = AutoState.SEARCH_SPIN;
                    }
                    break;
            }

            telemetry.addData("Auto State", state.toString());
            telemetry.addData("Robot X", "%.1f", currentX);
            telemetry.addData("Robot Y", "%.1f", currentY);
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(currentH));
            telemetry.update();
        }
        
        eater.stop();
        shooter.stop();
        vision.stop();
    }
}
