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

    private DcMotorEx eaterMotor;
    private DcMotorEx transferMotor;

    private CRServo sweeperLeft;
    private CRServo sweeperRight;

    private DistanceSensor colorSensorSecond;
    private DistanceSensor colorSensorFirst;
    private Telemetry telemetry;

    public enum State {
        STOP, INTAKE, REVERSE, MANUAL, FEEDING, INTAKE_ONLY, TRANSFER_ONLY
    }

    private State currentState = State.STOP;
    private double manualPower = 0.0;
    private boolean isBallDetected = false;
    private boolean isFull = false;
    private ElapsedTime feedTimer = new ElapsedTime();

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        eaterMotor = hardwareMap.get(DcMotorEx.class, "eater1");
        transferMotor = hardwareMap.get(DcMotorEx.class, "eater2");
        
        sweeperLeft = hardwareMap.get(CRServo.class, "sweeperLeft");
        sweeperRight = hardwareMap.get(CRServo.class, "sweeperRight");
        sweeperRight.setDirection(DcMotorSimple.Direction.REVERSE);

        colorSensorSecond = hardwareMap.get(DistanceSensor.class, "colorSensor");
        colorSensorFirst = hardwareMap.get(DistanceSensor.class, "colorSensor2");

        eaterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        eaterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        stopEater();
    }

    @Override
    public void update() {
        double intakePower = 0;
        double transferPower = 0;

        double distanceCmSecond = colorSensorSecond.getDistance(DistanceUnit.CM);
        double distanceCmFirst = colorSensorFirst.getDistance(DistanceUnit.CM);

        if (Math.max(distanceCmFirst, distanceCmSecond) > Constants.EATER_BALL_DISTANCE_CM + 2.0) {
            isBallDetected = false;
            isFull = false;
        }

        if (currentState == State.INTAKE && Math.min(distanceCmFirst, distanceCmSecond) < Constants.EATER_BALL_DISTANCE_CM){
            currentState = State.INTAKE_ONLY;
            isBallDetected = true; 
        }

        if (currentState == State.INTAKE_ONLY && Math.max(distanceCmFirst, distanceCmSecond) < Constants.EATER_BALL_DISTANCE_CM){
            isFull = true;
        }


            if (currentState == State.FEEDING) {
            if (feedTimer.milliseconds() > Constants.EATER_FEED_TIME_MS) {
                currentState = State.STOP;
            }
        }

        switch (currentState) {
            case INTAKE:
            case FEEDING:
                intakePower = Constants.EATER_POWER;
                transferPower = Constants.EATER_POWER;
                break;
            case INTAKE_ONLY:
                intakePower = Constants.EATER_POWER;
                transferPower = 0;
                break;
            case TRANSFER_ONLY:
                intakePower = 0;
                transferPower = Constants.EATER_POWER;
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

        eaterMotor.setPower(intakePower);
        transferMotor.setPower(transferPower);

        sweeperLeft.setPower(intakePower);
        sweeperRight.setPower(intakePower);

        telemetry.addData("Eater State", currentState);
        telemetry.addData("Distance (cm)", distanceCmSecond);
        telemetry.addData("Ball Detected", isBallDetected);
    }

    @Override
    public void stop() {
        stopEater();
    }

    public void runIntake() {
        if (currentState == State.FEEDING) {
            return; 
        }
        
        if (isBallDetected) {
            currentState = State.INTAKE_ONLY;
        } else if (currentState != State.INTAKE) {
            currentState = State.INTAKE;
        }
    }

    public void runReverse() {
        currentState = State.REVERSE;
    }

    public void stopEater() {
        if (currentState != State.FEEDING) {
            currentState = State.STOP;
        }
    }
    
    public void startIntake() {
        // 이전 사이클의 감지 상태를 지우고 무조건 인테이크부터 시작합니다.
        isBallDetected = false;
        currentState = State.INTAKE;
    }

    public void runIntakeOnly(){
        currentState = State.INTAKE_ONLY;
        update();
    }

    public void runTransferOnly(){
        currentState = State.TRANSFER_ONLY;
        update();
    }
    public void forceStopEater() {
        // 급탄 중이어도 무조건 정지합니다.
        currentState = State.STOP;
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
    public boolean isFull() {
        return isFull;
    }

    public Action runIntakeAction() {
        return new InstantAction(() -> runIntake());
    }

    public Action feedToShooterAction() {
        return new InstantAction(() -> feedToShooter());
    }

    public Action updateAction() {
        return new InstantAction(() -> update());
    }

    public Action startIntakeAction() {
        return new InstantAction(() -> startIntake());
    }

    public Action stopEaterAction() {
        return new InstantAction(() -> stopEater());
    }

    public Action runIntakeOnlyAction(){
        return new InstantAction(() -> runIntakeOnly());
    }

    public Action runTransferOnlyAction(){
        return new InstantAction(() -> runTransferOnly());
    }

    public Action forceStopEaterAction() {
        return new InstantAction(() -> forceStopEater());
    }

    public Action waitUntilFeedDoneAction() {
        return packet -> currentState == State.FEEDING;   // true = still running
    }
}
