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

@Autonomous(name = "Zone 1 AI (Hunt & Shoot)", group = "Auto")
public class AutoZone1_AI extends LinearOpMode {

    private enum AIState {
        SEARCHING_BALL,
        PURSUING_BALL,
        AIMING_GOAL,
        SHOOTING_MACRO
    }

    private enum ShootState {
        SPOOLING,
        FEED_1,
        DELAY,
        FEED_2
    }

    @Override
    public void runOpMode() {
        // 시작 위치 (0,0) 방향 0도 세팅
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Eater eater = new Eater();
        Shooter shooter = new Shooter();
        Vision vision = new Vision();

        eater.init(hardwareMap, telemetry);
        shooter.init(hardwareMap, telemetry);
        vision.init(hardwareMap, telemetry);

        telemetry.addLine("🚀 AI Auto Ready!");
        telemetry.addLine("Pipeline 0: Goal | Pipeline 1: Ball");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        eater.start();
        shooter.start();
        vision.start();

        AIState currentState = AIState.SEARCHING_BALL;
        ShootState shootState = ShootState.SPOOLING;
        ElapsedTime stateTimer = new ElapsedTime();
        ElapsedTime matchTimer = new ElapsedTime();

        // 1분 30초(최대 85초) 동안 무한 AI 루프 가동
        while (opModeIsActive() && matchTimer.seconds() < 85) {
            
            // 모든 부품 및 현재 로봇 위치(오도메트리) 최신화
            eater.update();
            shooter.update();
            vision.update();
            drive.updatePoseEstimate();
            
            Pose2d pose = drive.pose;

            switch (currentState) {
                case SEARCHING_BALL:
                    vision.setPipeline(1); // 공 찾기 모드
                    eater.runIntake(); // 공 먹기 시작

                    // 구역 경계(70x70) 이탈 방지 (여유를 두어 60인치 선에서 돌아옴)
                    if (pose.position.x > 60 || pose.position.x < -10 || pose.position.y > 60 || pose.position.y < -60) {
                        // 경계를 벗어나려 하면 뒤로 후진하며 꺾음
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-0.4, 0), 0.5));
                    } else {
                        // 천천히 전진하며 주변을 훑음 (크게 원을 그리며 탐색)
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.2, 0), 0.4));
                    }

                    // 공이 시야에 들어왔다!
                    if (vision.hasTarget()) {
                        currentState = AIState.PURSUING_BALL;
                    }
                    
                    // 운 좋게 공을 하나 삼켰다면 즉시 발사 모드로
                    if (eater.isBallDetected()) {
                        currentState = AIState.AIMING_GOAL;
                    }
                    break;

                case PURSUING_BALL:
                    vision.setPipeline(1);
                    eater.runIntake();
                    
                    if (vision.hasTarget()) {
                        // 공을 향해 조향하며 돌진
                        double turn = -vision.getTx() * 0.02; 
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.5, 0), turn));
                    } else {
                        // 공을 놓침 -> 다시 탐색 모드로
                        currentState = AIState.SEARCHING_BALL;
                    }

                    // 돌진하다가 공을 먹음
                    if (eater.isBallDetected()) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        currentState = AIState.AIMING_GOAL;
                    }
                    break;

                case AIMING_GOAL:
                    vision.setPipeline(0); // 골대 파이프라인으로 긴급 전환!
                    eater.stopEater(); // 먹은 공이 튀어나가지 않게 정지
                    
                    if (vision.hasTarget()) {
                        double turn = -vision.getTx() * Constants.VISION_TURN_KP;
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), turn));
                        
                        // 정조준 완료
                        if (Math.abs(vision.getTx()) < 1.5) {
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                            currentState = AIState.SHOOTING_MACRO;
                            shootState = ShootState.SPOOLING;
                            shooter.runShooter(Constants.FAR_SHOOT_VELOCITY);
                            stateTimer.reset();
                        }
                    } else {
                        // 골대가 안 보이면 제자리 회전하며 골대 찾기
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0.3));
                    }
                    break;

                case SHOOTING_MACRO:
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); // 정지 상태 유지
                    
                    switch (shootState) {
                        case SPOOLING:
                            if (stateTimer.milliseconds() > Constants.SHOOTER_SPOOL_TIME_MS) {
                                eater.feedToShooter();
                                stateTimer.reset();
                                shootState = ShootState.FEED_1;
                            }
                            break;
                        case FEED_1:
                            if (stateTimer.milliseconds() > Constants.EATER_FEED_TIME_MS) {
                                eater.stopEater();
                                stateTimer.reset();
                                shootState = ShootState.DELAY;
                            }
                            break;
                        case DELAY:
                            if (stateTimer.milliseconds() > Constants.RAPID_FIRE_DELAY_FAR_MS) {
                                eater.feedToShooter();
                                stateTimer.reset();
                                shootState = ShootState.FEED_2;
                            }
                            break;
                        case FEED_2:
                            if (stateTimer.milliseconds() > Constants.EATER_FEED_TIME_MS) {
                                shooter.stopShooter();
                                eater.stopEater();
                                // 발사 끝! 다시 공 주우러 출발
                                currentState = AIState.SEARCHING_BALL; 
                            }
                            break;
                    }
                    break;
            }

            telemetry.addData("AI State", currentState);
            telemetry.addData("Shoot State", shootState);
            telemetry.addData("X", pose.position.x);
            telemetry.addData("Y", pose.position.y);
            telemetry.addData("Time Left", 85 - matchTimer.seconds());
            telemetry.update();
        }

        // 경기 종료 5초 전 멈춤
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        eater.stop();
        shooter.stop();
        vision.stop();
    }
}
