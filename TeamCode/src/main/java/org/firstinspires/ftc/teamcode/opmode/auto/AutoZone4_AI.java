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

@Autonomous(name = "Zone 4 (Auto)", group = "Auto")
public class AutoZone4_AI extends LinearOpMode {

    public enum AutoState {
        MOVE_TO_SEARCH_START, // 시작 시 또는 사격 후 탐색 시작 명당 위치로 이동
        SEARCH_SPIN,    
        SEARCH_MOVE,    
        TRACKING_BALL,  
        BLIND_PURSUIT,  
        ABORT_REVERSE,  
        MOVE_TO_SHOOT,  
        TURN_TO_SHOOT,  
        AIMING_GOAL,    
        SHOOTING,       
        JAM_RECOVERY,    // 슈터 과전류 발생 시 강제 복구 상태
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
        
        // Zone 4 전용 목표 골대 에이프릴 태그 ID 설정
        vision.setTargetTagId(Constants.ZONE4_TARGET_TAG_ID);

        Vector2d[] safeWaypoints = {
            new Vector2d(25, 25),
            new Vector2d(40, 15),
            new Vector2d(45, 40),
            new Vector2d(20, 45),
            new Vector2d(15, 20)
        };
        int waypointIndex = 0;

        AutoState state = AutoState.MOVE_TO_SEARCH_START;
        double searchStartH = 0.0;
        double blindPursuitHeading = 0.0;
        
        ElapsedTime macroTimer = new ElapsedTime();
        ElapsedTime abortTimer = new ElapsedTime(); 
        
        boolean isBlind = false;
        ElapsedTime blindTimer = new ElapsedTime();
        
        int shootStep = 0;
        int jamRetryCount = 0; // 슈터 과전류 발생 시 재시도 횟수
        boolean lastBallDetected = false;
        
        double escapeFieldX = 0;
        double escapeFieldY = 0;
        boolean isFieldEscape = false;

        telemetry.addLine("Auto Zone 4 Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        eater.start();
        shooter.start();
        vision.start();
        
        // 초기 적재된 공 유무 확인 및 발사 로직
        eater.update(); // 센서 상태 최초 갱신
        if (eater.isBallDetected()) { // 1개든 2개든 공이 있으면 무조건 사격부터 하러 감
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

            if (isBlind && blindTimer.milliseconds() > 1500) {
                isBlind = false; // 1.5초 지나면 눈을 뜸
            }

            boolean currentBallDetected = eater.isBallDetected();
            if (currentBallDetected && !lastBallDetected) {
                // 방금 공을 하나 먹었음! 두 번째 공을 찾기 위한 추적 타임아웃 타이머를 리셋
                abortTimer.reset();
            }
            lastBallDetected = currentBallDetected;

            double currentX = drive.localizer.getPose().position.x;
            double currentY = drive.localizer.getPose().position.y;
            double currentH = drive.localizer.getPose().heading.toDouble();

            // [글로벌 안전장치] 공을 1개라도 먹었을 때:
            // 사냥 관련 상태(발사하러 가기 전)에 있다면 즉시 모든 행동을 취소하고 사격 위치로 이동합니다.
            if (state.ordinal() < AutoState.MOVE_TO_SHOOT.ordinal()) {
                if (eater.isBallDetected()) { // 1개라도 먹으면 즉시 쏘러 감
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    eater.stopEater(); // 인테이크/트랜스퍼 정지 (공이 더 밀려들어가지 않도록)
                    state = AutoState.MOVE_TO_SHOOT;
                }
            }

            switch (state) {
                case MOVE_TO_SEARCH_START:
                    // 사격 후, 또는 초기 시작 시 지정된 탐색 시작 위치(오른쪽 앞 모서리 등)로 이동
                    if (vision.hasTarget() && !isBlind) {
                        abortTimer.reset();
                        state = AutoState.TRACKING_BALL;
                        break;
                    }
                    double startTargetX = Constants.ZONE4_SEARCH_START_X;
                    double startTargetY = Constants.ZONE4_SEARCH_START_Y;
                    
                    double startMoveErrorX = startTargetX - currentX;
                    double startMoveErrorY = startTargetY - currentY;
                    double startMoveDistance = Math.hypot(startMoveErrorX, startMoveErrorY);
                    
                    double startTurnError = Math.toRadians(Constants.ZONE4_SEARCH_START_HEADING) - currentH;
                    while (startTurnError > Math.PI) startTurnError -= 2 * Math.PI;
                    while (startTurnError < -Math.PI) startTurnError += 2 * Math.PI;
                    
                    // 이동 및 회전이 거의 끝나면 스캔(SEARCH_SPIN) 시작
                    if (startMoveDistance < 2.0 && Math.abs(startTurnError) < Math.toRadians(5)) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        searchStartH = currentH;
                        state = AutoState.SEARCH_SPIN;
                        break;
                    }
                    
                    double sc = Math.cos(-currentH);
                    double ss = Math.sin(-currentH);
                    double sRobX = startMoveErrorX * sc - startMoveErrorY * ss;
                    double sRobY = startMoveErrorX * ss + startMoveErrorY * sc;

                    double sPowerX = sRobX * Constants.AUTO_DRIVE_KP;
                    double sPowerY = sRobY * Constants.AUTO_DRIVE_KP;
                    
                    double sMagnitude = Math.hypot(sPowerX, sPowerY);
                    if (sMagnitude > Constants.AUTO_MAX_DRIVE_SPEED) {
                        sPowerX = (sPowerX / sMagnitude) * Constants.AUTO_MAX_DRIVE_SPEED;
                        sPowerY = (sPowerY / sMagnitude) * Constants.AUTO_MAX_DRIVE_SPEED;
                    }
                    
                    double sTurnPower = startTurnError * 0.5;
                    if (sTurnPower > 0.3) sTurnPower = 0.3;
                    if (sTurnPower < -0.3) sTurnPower = -0.3;
                    if (Math.abs(sTurnPower) < Constants.AUTO_AIM_MIN_SPEED && sTurnPower != 0) {
                        sTurnPower = Math.signum(sTurnPower) * Constants.AUTO_AIM_MIN_SPEED;
                    }

                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(sPowerX, sPowerY), sTurnPower
                    ));
                    break;

                case SEARCH_SPIN:
                    if (vision.hasTarget() && !isBlind) {
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
                    if (vision.hasTarget() && !isBlind) {
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

                    double powerX = robotX * Constants.AUTO_DRIVE_KP;
                    double powerY = robotY * Constants.AUTO_DRIVE_KP;
                    
                    // 미끄러짐 방지를 위한 최대 속도 제한
                    double magnitude = Math.hypot(powerX, powerY);
                    if (magnitude > Constants.AUTO_MAX_DRIVE_SPEED) {
                        powerX = (powerX / magnitude) * Constants.AUTO_MAX_DRIVE_SPEED;
                        powerY = (powerY / magnitude) * Constants.AUTO_MAX_DRIVE_SPEED;
                    }

                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(powerX, powerY), 0
                    ));
                    break;

                case TRACKING_BALL:

                    if (abortTimer.milliseconds() > Constants.AUTO_PURSUIT_TIMEOUT_MS) {
                        isFieldEscape = false;
                        state = AutoState.ABORT_REVERSE;
                        macroTimer.reset();
                        break;
                    }
                    
                    // Zone 4 물리적 한계: 앞쪽(+X)과 오른쪽(-Y)은 벽. 뒷쪽(-X)과 왼쪽(+Y)은 열린 공간.
                    if (currentX > Constants.ZONE_LENGTH_X + Constants.AUTO_WALL_EXTENSION_INCHES) { // 앞쪽 (벽)
                        escapeFieldX = -Constants.AUTO_FORWARD_SPEED; escapeFieldY = 0; isFieldEscape = true;
                        state = AutoState.ABORT_REVERSE; macroTimer.reset(); break;
                    } else if (currentX < -Constants.AUTO_OPEN_EXTENSION_INCHES) { // 뒤쪽 (열린 공간)
                        escapeFieldX = Constants.AUTO_FORWARD_SPEED; escapeFieldY = 0; isFieldEscape = true;
                        state = AutoState.ABORT_REVERSE; macroTimer.reset(); break;
                    } else if (currentY > Constants.AUTO_OPEN_EXTENSION_INCHES) { // 왼쪽 (열린 공간)
                        escapeFieldX = 0; escapeFieldY = -Constants.AUTO_FORWARD_SPEED; isFieldEscape = true;
                        state = AutoState.ABORT_REVERSE; macroTimer.reset(); break;
                    } else if (currentY < -Constants.ZONE_WIDTH_Y - Constants.AUTO_WALL_EXTENSION_INCHES) { // 오른쪽 (벽)
                        escapeFieldX = 0; escapeFieldY = Constants.AUTO_FORWARD_SPEED; isFieldEscape = true;
                        state = AutoState.ABORT_REVERSE; macroTimer.reset(); break;
                    }

                    if (!vision.hasTarget()) {
                        blindPursuitHeading = currentH;
                        state = AutoState.BLIND_PURSUIT;
                        break;
                    }

                    double txBall = vision.getTx();
                    double trackTurn = -txBall * Constants.BALL_VISION_TURN_KP;
                    // 공을 쫓아갈 때 회전(조향)이 너무 급격하지 않도록 최대 속도를 0.2로 제한
                    if (trackTurn > 0.2) trackTurn = 0.2;
                    if (trackTurn < -0.2) trackTurn = -0.2;
                    
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(Constants.AUTO_FORWARD_SPEED, 0), trackTurn
                    ));
                    break;

                case BLIND_PURSUIT:
                    
                    if (vision.hasTarget()) {
                        state = AutoState.TRACKING_BALL;
                        break;
                    }

                    if (abortTimer.milliseconds() > Constants.AUTO_PURSUIT_TIMEOUT_MS) {
                        isFieldEscape = false;
                        state = AutoState.ABORT_REVERSE;
                        macroTimer.reset();
                        break;
                    }

                    if (currentX > Constants.ZONE_LENGTH_X + Constants.AUTO_WALL_EXTENSION_INCHES) { 
                        escapeFieldX = -Constants.AUTO_FORWARD_SPEED; escapeFieldY = 0; isFieldEscape = true;
                        state = AutoState.ABORT_REVERSE; macroTimer.reset(); break;
                    } else if (currentX < -Constants.AUTO_OPEN_EXTENSION_INCHES) { 
                        escapeFieldX = Constants.AUTO_FORWARD_SPEED; escapeFieldY = 0; isFieldEscape = true;
                        state = AutoState.ABORT_REVERSE; macroTimer.reset(); break;
                    } else if (currentY > Constants.AUTO_OPEN_EXTENSION_INCHES) { 
                        escapeFieldX = 0; escapeFieldY = -Constants.AUTO_FORWARD_SPEED; isFieldEscape = true;
                        state = AutoState.ABORT_REVERSE; macroTimer.reset(); break;
                    } else if (currentY < -Constants.ZONE_WIDTH_Y - Constants.AUTO_WALL_EXTENSION_INCHES) { 
                        escapeFieldX = 0; escapeFieldY = Constants.AUTO_FORWARD_SPEED; isFieldEscape = true;
                        state = AutoState.ABORT_REVERSE; macroTimer.reset(); break;
                    }

                    double blindTurn = (blindPursuitHeading - currentH) * 0.5;
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(Constants.AUTO_FORWARD_SPEED, 0), blindTurn
                    ));
                    break;

                case ABORT_REVERSE:
                    if (isFieldEscape) {
                        double escCos = Math.cos(-currentH);
                        double escSin = Math.sin(-currentH);
                        double escRobX = escapeFieldX * escCos - escapeFieldY * escSin;
                        double escRobY = escapeFieldX * escSin + escapeFieldY * escCos;
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(escRobX, escRobY), 0));
                    } else {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-Constants.AUTO_FORWARD_SPEED, 0), 0));
                    }
                    
                    if (macroTimer.milliseconds() > Constants.AUTO_ABORT_REVERSE_TIME_MS) {
                        macroTimer.reset(); 
                        // 포기했으면 배가 텅 빈 상태이므로(1개라도 먹었다면 글로벌 감시가 채갔음) 무조건 사냥 복귀
                        isBlind = true;
                        blindTimer.reset();
                        searchStartH = currentH;
                        state = AutoState.SEARCH_SPIN;
                    }
                    break;

                case MOVE_TO_SHOOT:
                    // 1단계: 사격 위치로 먼저 이동 (이동 중에는 0도 정면 유지)
                    double shootTargetX = Constants.ZONE4_SHOOT_POSE_X;
                    double shootTargetY = Constants.ZONE4_SHOOT_POSE_Y;
                    
                    double moveErrorX = shootTargetX - currentX;
                    double moveErrorY = shootTargetY - currentY;
                    double moveDistance = Math.hypot(moveErrorX, moveErrorY);
                    
                    double moveTurnError = 0.0 - currentH;
                    while (moveTurnError > Math.PI) moveTurnError -= 2 * Math.PI;
                    while (moveTurnError < -Math.PI) moveTurnError += 2 * Math.PI;

                    // 이동이 거의 끝나면(오차 2인치 이내) 회전 단계로 넘어감
                    if (moveDistance < 2.0) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        macroTimer.reset(); // TURN_TO_SHOOT 타임아웃용 타이머 시작
                        state = AutoState.TURN_TO_SHOOT;
                        break;
                    }

                    double c = Math.cos(-currentH);
                    double s = Math.sin(-currentH);
                    double robX = moveErrorX * c - moveErrorY * s;
                    double robY = moveErrorX * s + moveErrorY * c;

                    double movePowerX = robX * Constants.AUTO_DRIVE_KP;
                    double movePowerY = robY * Constants.AUTO_DRIVE_KP;
                    
                    // 미끄러짐 방지를 위한 최대 속도 제한
                    double moveMagnitude = Math.hypot(movePowerX, movePowerY);
                    if (moveMagnitude > Constants.AUTO_MAX_DRIVE_SPEED) {
                        movePowerX = (movePowerX / moveMagnitude) * Constants.AUTO_MAX_DRIVE_SPEED;
                        movePowerY = (movePowerY / moveMagnitude) * Constants.AUTO_MAX_DRIVE_SPEED;
                    }

                    // 이동 중 회전은 0도 유지 (급회전 방지를 위해 속도 클램핑)
                    double moveTurnPower = moveTurnError * 0.5;
                    if (moveTurnPower > 0.3) moveTurnPower = 0.3;
                    if (moveTurnPower < -0.3) moveTurnPower = -0.3;

                    // 이동 수행
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(movePowerX, movePowerY), moveTurnPower
                    ));
                    break;
                    
                case TURN_TO_SHOOT:
                    // 2단계: 도착 후 지정된 기본 각도 방향으로 제자리 회전
                    double shootTargetH = Math.toRadians(Constants.ZONE4_SHOOT_HEADING_DEG);
                    double turnError = shootTargetH - currentH;
                    while (turnError > Math.PI) turnError -= 2 * Math.PI;
                    while (turnError < -Math.PI) turnError += 2 * Math.PI;

                    // 회전이 완료되면(오차 2도 이내) 비전 조준 상태로 넘어감
                    if (Math.abs(turnError) < Math.toRadians(2)) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        vision.setPipeline(0); 
                        macroTimer.reset(); 
                        state = AutoState.AIMING_GOAL;
                        break;
                    }
                    
                    // [타임아웃] 오도메트리가 틀어져도 2초가 지나면 강제로 비전 조준 단계로 넘어감
                    if (macroTimer.milliseconds() > 2000) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        vision.setPipeline(0);
                        macroTimer.reset();
                        state = AutoState.AIMING_GOAL;
                        break;
                    }
                    
                    // 갑작스런 급발진 회전을 막기 위해 1.5 대신 0.5 사용
                    double autoTurn = turnError * 0.5;
                    if (Math.abs(autoTurn) < Constants.AUTO_AIM_MIN_SPEED) {
                        autoTurn = Math.signum(autoTurn) * Constants.AUTO_AIM_MIN_SPEED;
                    }
                    
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), autoTurn));
                    break;

                case AIMING_GOAL:
                    if (macroTimer.milliseconds() < 400) {
                        // 골대(Limelight 파이프라인 0) 모드로 전환되는 동안에는 센서를 믿지 않고 기본 사격 각도 고정
                        double aimTurnError = Math.toRadians(Constants.ZONE4_SHOOT_HEADING_DEG) - currentH;
                        while (aimTurnError > Math.PI) aimTurnError -= 2 * Math.PI;
                        while (aimTurnError < -Math.PI) aimTurnError += 2 * Math.PI;
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), aimTurnError * 1.0));
                    }
                    else if (vision.hasTarget()) {
                        // 영점 조절: 오른쪽으로 빗나갈 경우 오프셋을 빼주면 로봇이 더 왼쪽을 조준하게 됨
                        double txGoal = vision.getTx() - Constants.ZONE4_SHOOT_TX_OFFSET;
                        // 오차가 작거나 너무 오래 걸리면(2초 초과) 일단 발사
                        if (Math.abs(txGoal) < 1.5 || macroTimer.milliseconds() > 2400) { 
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                            shooter.runShooter(Constants.FAR_SHOOT_VELOCITY);
                            macroTimer.reset();
                            shootStep = 1;
                            state = AutoState.SHOOTING;
                        } else {
                            double aimTurn = -txGoal * Constants.ZONE2_4_VISION_TURN_KP;
                            
                            // 화면 가장자리에서 발견했을 때 파워가 폭주하여 급가속하는 현상 방지 (최대 0.3으로 제한)
                            if (aimTurn > 0.3) aimTurn = 0.3;
                            if (aimTurn < -0.3) aimTurn = -0.3;

                            // 마찰력을 이기고 움직이기 위한 최소 파워(Feedforward) 설정
                            if (Math.abs(aimTurn) < Constants.AUTO_AIM_MIN_SPEED && aimTurn != 0) {
                                aimTurn = Math.signum(aimTurn) * Constants.AUTO_AIM_MIN_SPEED;
                            }
                            
                            drive.setDrivePowers(new PoseVelocity2d(
                                    new Vector2d(0, 0), aimTurn
                            ));
                        }
                    } else {
                        // 태그가 안 보임: 오도메트리에 의존하지 않고 좌우 스윕하며 태그 탐색
                        if (macroTimer.milliseconds() > 4000) {
                            // 4초 이상 태그를 못 찾으면 강제 발사
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                            shooter.runShooter(Constants.CLOSE_SHOOT_VELOCITY);
                            macroTimer.reset();
                            shootStep = 1;
                            state = AutoState.SHOOTING;
                        } else if (macroTimer.milliseconds() > 400) {
                            // 1초 주기로 좌우 교대 스윕 (오도메트리 무시, 카메라로 직접 태그 탐색)
                            double sweepDir = ((int)((macroTimer.milliseconds() - 400) / 1000)) % 2 == 0 ? 1.0 : -1.0;
                            drive.setDrivePowers(new PoseVelocity2d(
                                    new Vector2d(0, 0), sweepDir * Constants.AUTO_AIM_MIN_SPEED
                            ));
                        }
                    }
                    break;

                case SHOOTING:
                    // 단발 사격(1개 먹고 1개 쏘기)으로 단순화
                    // 1단계: 플라이휠 예열 대기 후 공 피딩 (과전류 걸림 감지 탑재)
                    if (shootStep == 1) {
                        // 모터 기동 인러시 전류 피크(300ms) 회피 후 전류 감지
                        if (macroTimer.milliseconds() > 300 && shooter.getCurrent() > Constants.AUTO_JAM_CURRENT_LIMIT) {
                            if (jamRetryCount < 2) {
                                jamRetryCount++;
                                shooter.stopShooter();
                                eater.runJamRecovery(); // 트랜스퍼는 역회전, 인테이크는 정회전하여 공 뱉음 방지
                                macroTimer.reset();
                                state = AutoState.JAM_RECOVERY;
                                break;
                            } else {
                                // 2번 시도했는데도 계속 걸리면 억지로 밀어넣어 사격 시도
                                eater.feedToShooter();
                                macroTimer.reset();
                                shootStep = 2;
                            }
                        }
                        
                        if (macroTimer.milliseconds() > Constants.SHOOTER_SPOOL_TIME_MS) {
                            eater.feedToShooter();
                            macroTimer.reset();
                            shootStep = 2;
                        }
                    } 
                    // 2단계: 공 피딩 완료 후 사격 종료 및 즉시 다음 사냥 준비
                    else if (shootStep == 2 && macroTimer.milliseconds() > Constants.EATER_FEED_TIME_MS) {
                        shooter.stopShooter();
                        eater.forceStopEater(); // 피딩 강제 종료
                        
                        vision.setPipeline(1);
                        eater.startIntake(); // 다음 사이클을 위해 인테이크 즉시 가동
                        
                        jamRetryCount = 0; // 사격 완료 시 걸림 복구 카운트 리셋
                        macroTimer.reset();
                        state = AutoState.PIPELINE_WAIT;
                    }
                    break;

                case JAM_RECOVERY:
                    // 1단계: 슈터에 걸린 공이 센서에서 완전히 빠져나갈 때까지 트랜스퍼 역회전 (인테이크는 정회전)
                    if (shootStep == 1) {
                        // 최소 역회전 시간(300ms)을 만족하고 센서에서도 공이 빠져나갔을 때 다음 단계로 진입
                        if (macroTimer.milliseconds() > Constants.AUTO_JAM_REVERSE_TIME_MS && !eater.isBallDetected()) {
                            eater.startIntake(); // 역회전 중지하고 정상 흡입(Intake + Transfer 전진) 시작
                            macroTimer.reset();
                            shootStep = 2;
                        } else if (macroTimer.milliseconds() > 800) {
                            // 센서 감지와 무관하게 800ms 이상 역회전했으면 강제로 다시 먹기 시작
                            eater.startIntake();
                            macroTimer.reset();
                            shootStep = 2;
                        }
                    }
                    // 2단계: 공이 다시 첫 번째 센서에 걸릴 때까지 정상 인테이크+트랜스퍼 작동
                    else if (shootStep == 2) {
                        if (eater.isBallDetected()) {
                            eater.forceStopEater(); // 인테이크 및 트랜스퍼 정지
                            shooter.runShooter(Constants.CLOSE_SHOOT_VELOCITY); // 슈터 예열 재가동
                            macroTimer.reset();
                            shootStep = 1; // SHOOTING 상태의 1단계(예열)로 설정
                            state = AutoState.SHOOTING;
                        }
                    }
                    break;

                case PIPELINE_WAIT:
                    if (macroTimer.milliseconds() > 200) {
                        state = AutoState.MOVE_TO_SEARCH_START;
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
