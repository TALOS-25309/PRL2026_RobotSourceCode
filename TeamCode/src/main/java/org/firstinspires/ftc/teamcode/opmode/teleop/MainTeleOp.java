package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.part.Constants;
import org.firstinspires.ftc.teamcode.part.Drive;
import org.firstinspires.ftc.teamcode.part.eater.Eater;
import org.firstinspires.ftc.teamcode.part.shooter.Shooter;
import org.firstinspires.ftc.teamcode.part.Vision;

@TeleOp(name = "1. Main TeleOp", group = "Main")
public class MainTeleOp extends LinearOpMode {

    private Drive drive;
    private Eater eater;
    private Shooter shooter;
    private Vision vision;

    private boolean lastBallDetected = false;

    // --- 스마트 슈팅 매크로 상태 변수들 ---
    private enum ShootMacroState {
        IDLE,                   // 대기 중
        SPOOLING,               // 발사 전 목표 RPM 도달 대기
        FEEDING,                // 공을 밀어넣는 중
        WAITING_FOR_NEXT_RAPID  // 연사 중 다음 공 발사 대기
    }
    
    private ShootMacroState shootState = ShootMacroState.IDLE;
    private ElapsedTime macroTimer = new ElapsedTime();
    private boolean isRapidMode = false;
    private double currentMacroVelocity = 0.0;
    private double currentMacroRapidDelay = 0.0;

    @Override
    public void runOpMode() {
        drive = new Drive();
        eater = new Eater();
        shooter = new Shooter();
        vision = new Vision();

        drive.init(hardwareMap, telemetry);
        eater.init(hardwareMap, telemetry);
        shooter.init(hardwareMap, telemetry);
        vision.init(hardwareMap, telemetry);

        telemetry.addLine("Main TeleOp Ready!");
        telemetry.addLine("=================================");
        telemetry.addLine("[Drive] Left Stick: Move | Right Stick X: Turn");
        telemetry.addLine("[Eater] LT: Intake | LB: Spit");
        telemetry.addLine("[Shoot - Far]   Y (Single) | RT (Rapid)");
        telemetry.addLine("[Shoot - Close] A (Single) | RB (Rapid)");
        telemetry.addLine("[Vision] Hold X to Auto-Aim");
        telemetry.addLine("=================================");
        telemetry.update();

        waitForStart();

        drive.start();
        eater.start();
        shooter.start();
        vision.start();

        while (opModeIsActive()) {
            
            // ---------------------------------
            // A. 주행 (Drive) 및 오토 에임 제어
            // ---------------------------------
            double forward = -gamepad1.left_stick_y * Constants.DRIVE_SPEED_MULTIPLIER;
            double strafe = -gamepad1.left_stick_x * Constants.DRIVE_SPEED_MULTIPLIER;
            double turn = -gamepad1.right_stick_x * Constants.DRIVE_SPEED_MULTIPLIER;
            
            // X 버튼(또는 선택한 버튼)을 누르고 있고 타겟이 보인다면 오토 에임 발동!
            if (gamepad1.x && vision.hasTarget()) {
                // tx값이 양수면 타겟이 오른쪽에 있음 -> 오른쪽으로 회전 (turn을 음수로)
                // tx값이 음수면 타겟이 왼쪽에 있음 -> 왼쪽으로 회전 (turn을 양수로)
                turn = -vision.getTx() * Constants.VISION_TURN_KP;
            }
            
            drive.setDrivePower(new PoseVelocity2d(new Vector2d(forward, strafe), turn));

            // ---------------------------------
            // B. 이터(Intake & Transfer) 제어
            // ---------------------------------
            // 매크로가 작동 중일 때(FEEDING 등)는 이터가 강제 제어되므로, 
            // 수동 흡입(LT)은 매크로가 IDLE이거나 예열(SPOOLING) 중일 때만 작동하도록 합니다.
            if (shootState == ShootMacroState.IDLE || shootState == ShootMacroState.SPOOLING) {
                if (gamepad1.left_trigger > 0.1) {
                    eater.runIntake(); 
                } else if (gamepad1.left_bumper) {
                    eater.runReverse();
                } else {
                    eater.stopEater();
                }
            }

            boolean currentBallDetected = eater.isBallDetected();
            if (currentBallDetected && !lastBallDetected) {
                gamepad1.rumble(500); 
            }
            lastBallDetected = currentBallDetected;


            // ---------------------------------
            // C. 스마트 슈팅 매크로 (원버튼 발사)
            // ---------------------------------
            boolean reqFarSingle = gamepad1.y;
            boolean reqFarRapid = gamepad1.right_trigger > 0.1;
            boolean reqCloseSingle = gamepad1.a;
            boolean reqCloseRapid = gamepad1.right_bumper;

            // 어떤 버튼을 눌렀는지에 따라 매크로 시작 조건 판단
            boolean triggerPressed = reqFarSingle || reqFarRapid || reqCloseSingle || reqCloseRapid;
            
            // 연사 버튼(RT, RB)을 누르고 있는지 여부
            boolean holdingRapid = reqFarRapid || reqCloseRapid;

            switch (shootState) {
                case IDLE:
                    if (triggerPressed) {
                        // 거리(속도 및 딜레이) 결정
                        if (reqFarSingle || reqFarRapid) {
                            currentMacroVelocity = Constants.FAR_SHOOT_VELOCITY;
                            currentMacroRapidDelay = Constants.RAPID_FIRE_DELAY_FAR_MS;
                        } else {
                            currentMacroVelocity = Constants.CLOSE_SHOOT_VELOCITY;
                            currentMacroRapidDelay = Constants.RAPID_FIRE_DELAY_CLOSE_MS;
                        }
                        // 모드 결정
                        isRapidMode = reqFarRapid || reqCloseRapid;
                        
                        // 슈터 작동 시작 및 타이머 리셋
                        shooter.runShooter(currentMacroVelocity);
                        macroTimer.reset();
                        shootState = ShootMacroState.SPOOLING;
                    } else {
                        shooter.stopShooter();
                    }
                    break;

                case SPOOLING:
                    // 예열 중일 때 연사 모드라면, 버튼에서 손을 떼면 즉시 취소
                    if (isRapidMode && !holdingRapid) {
                        shootState = ShootMacroState.IDLE;
                        break;
                    }
                    
                    // 정해진 예열 시간(0.4초)이 지나면 발사(Feed) 시작
                    if (macroTimer.milliseconds() >= Constants.SHOOTER_SPOOL_TIME_MS) {
                        eater.feedToShooter();
                        macroTimer.reset();
                        shootState = ShootMacroState.FEEDING;
                    }
                    break;

                case FEEDING:
                    // 연사 모드인데 버튼 떼면 취소 (피딩 중인 건 마저 끝나고 멈춤)
                    if (isRapidMode && !holdingRapid) {
                        isRapidMode = false; // 연사 취소, 이번 발사만 끝내고 IDLE로 가도록 설정
                    }

                    // 급탄 시간(0.35초)이 끝나면
                    if (macroTimer.milliseconds() >= Constants.EATER_FEED_TIME_MS) {
                        if (isRapidMode) {
                            // 연사 모드면 다음 발사를 위해 대기 상태로
                            macroTimer.reset();
                            shootState = ShootMacroState.WAITING_FOR_NEXT_RAPID;
                        } else {
                            // 단발 모드면 모든 매크로 종료
                            shootState = ShootMacroState.IDLE;
                        }
                    }
                    break;

                case WAITING_FOR_NEXT_RAPID:
                    // 대기 중에 버튼에서 손을 떼면 즉시 종료
                    if (!holdingRapid) {
                        shootState = ShootMacroState.IDLE;
                        break;
                    }
                    
                    // 설정된 연사 딜레이가 끝나면 다시 급탄(Feed)
                    if (macroTimer.milliseconds() >= currentMacroRapidDelay) {
                        eater.feedToShooter();
                        macroTimer.reset();
                        shootState = ShootMacroState.FEEDING;
                    }
                    break;
            }

            // ---------------------------------
            // D. 부품 업데이트
            // ---------------------------------
            vision.update();
            drive.update();
            eater.update();
            shooter.update();
            
            telemetry.addData("Macro State", shootState);
            telemetry.update();
        }

        drive.stop();
        eater.stop();
        shooter.stop();
        vision.stop();
    }
}
