package org.firstinspires.ftc.teamcode.part.eater;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.part.Constants;
import org.firstinspires.ftc.teamcode.part.Part;

public class Eater implements Part {

    // eater1 = 바닥 인테이크 (Floor Eater)
    // eater2 = 위쪽 트랜스퍼 (Transfer)
    private DcMotorEx eater1; 
    private DcMotorEx eater2; 
    
    // 양옆 스위퍼 (연속 회전 서보)
    private CRServo sweeperLeft;
    private CRServo sweeperRight;
    
    private DistanceSensor colorSensor; 
    private Telemetry telemetry;

    public enum State {
        STOP, INTAKE, REVERSE, MANUAL, FEEDING, INTAKE_ONLY
    }

    private State currentState = State.STOP;
    private double manualPower = 0.0;
    
    // 외부에 공이 감지되었는지 알려주기 위한 플래그 
    private boolean isBallDetected = false;
    
    private ElapsedTime feedTimer = new ElapsedTime();

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        eater1 = hardwareMap.get(DcMotorEx.class, "eater1"); // 바닥 인테이크
        eater2 = hardwareMap.get(DcMotorEx.class, "eater2"); // 위쪽 트랜스퍼
        
        sweeperLeft = hardwareMap.get(CRServo.class, "sweeperLeft");
        sweeperRight = hardwareMap.get(CRServo.class, "sweeperRight");
        
        // 오른쪽 스위퍼는 보통 장착 방향 때문에 역방향으로 돕니다. 필요시 반전시킵니다.
        sweeperRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
        colorSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");

        eater1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        eater2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // 정지 시 전기적 브레이크를 걸어 관성을 줄임
        eater1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        eater2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // 벨트 결합 구조에 따라 필요시 주석 해제하여 방향 반전
         eater2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void start() {
        stopEater();
    }

    @Override
    public void update() {
        double intakePower = 0;   // eater1 파워
        double transferPower = 0; // eater2 파워
        
        // 거리 센서로 공 거리 측정 (단위: cm)
        double distanceCm = colorSensor.getDistance(DistanceUnit.CM);
        
        // 센서 기반 볼 감지 상태 자동 리셋 로직 (공이 밖으로 빠지거나 쐈을 때)
        if (distanceCm > Constants.EATER_BALL_DISTANCE_CM + 2.0) {
            isBallDetected = false;
        }


        // 1. 자동 정지 (오토스탑) 로직
        if (currentState == State.INTAKE && distanceCm < Constants.EATER_BALL_DISTANCE_CM) {
            currentState = State.INTAKE_ONLY; // 감지 즉시 트랜스퍼 정지 및 2-Ball 수납 모드 전환
            isBallDetected = true; 
        }
        // 4. 급탄(Feed) 시퀀스 제어
        if (currentState == State.FEEDING) {
            if (feedTimer.milliseconds() > Constants.EATER_FEED_TIME_MS) {
                currentState = State.STOP;
                // 공이 날아갔으니 센서가 다시 멀어졌다고 감지할 때까지 대기
            }
        }
        
        // 상태별 모터 파워 독립 인가
        switch (currentState) {
            case INTAKE:
            case FEEDING: 
                // 둘 다 정방향 회전
                intakePower = Constants.EATER_POWER;
                transferPower = Constants.EATER_POWER;
                break;
            case INTAKE_ONLY: 
                // 트랜스퍼는 멈추고 인테이크만 회전 (두 번째 공 먹기)
                intakePower = Constants.EATER_POWER;
                transferPower = 0;
                break;
                
            case REVERSE:
                intakePower = Constants.EATER_REVERSE_POWER;
                transferPower = Constants.EATER_REVERSE_POWER;
                break;
                
            case MANUAL:
                intakePower = manualPower;
                transferPower = manualPower;
                break;
                
            case STOP:
            default:
                intakePower = 0;
                transferPower = 0;
                break;
        }

        eater1.setPower(intakePower);
        eater2.setPower(transferPower); 
        
        // 양옆 스위퍼는 항상 1번 인테이크(eater1)와 똑같이 움직임
        sweeperLeft.setPower(intakePower);
        sweeperRight.setPower(intakePower);

        telemetry.addData("Eater State", currentState);
        telemetry.addData("Distance (cm)", distanceCm);
        telemetry.addData("Ball Detected", isBallDetected);
    }

    @Override
    public void stop() {
        stopEater();
    }

    // --- 외부 제어 메서드 ---
    public void runIntake() {
        // 급탄 중일 때는 외부 명령(조종기)을 무시합니다.
        if (currentState == State.FEEDING) {
            return; 
        }
        
        if (isBallDetected) {
            // 이미 1번 공이 들어있다면 바닥의 인테이크 모터만 돕니다.
            currentState = State.INTAKE_ONLY;
        } else if (currentState != State.INTAKE) {
            // 공이 없으면 둘 다 돕니다.
            currentState = State.INTAKE;
        }
    }

    public void runReverse() {
        currentState = State.REVERSE;
    }

    public void stopEater() {
        // 발사 중일 때는 손을 떼도 강제로 멈추지 않고 시퀀스를 마저 끝냅니다.
        if (currentState != State.FEEDING) {
            currentState = State.STOP;
        }
    }
    
    public void feedToShooter() {
        if (currentState != State.FEEDING) {
            currentState = State.FEEDING;
            feedTimer.reset();
        }
    }

    public void manualEater(double power) {
        currentState = State.MANUAL;
        this.manualPower = power;
    }
    
    public boolean isBallDetected() {
        return isBallDetected;
    }

    // --- 자율주행(RoadRunner 1.0)용 Action 반환 메서드 ---
    public Action runIntakeAction() {
        return new InstantAction(() -> runIntake());
    }

    public Action feedToShooterAction() {
        return new InstantAction(() -> feedToShooter());
    }

    public Action stopEaterAction() {
        return new InstantAction(() -> stopEater());
    }
}
