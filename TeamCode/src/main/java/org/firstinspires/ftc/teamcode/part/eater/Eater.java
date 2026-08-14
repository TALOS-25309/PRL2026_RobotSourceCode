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

    private DcMotorEx eater1; 
    private DcMotorEx eater2; 
    
    private CRServo sweeperLeft;
    private CRServo sweeperRight;
    
    private DistanceSensor colorSensor; 
    private Telemetry telemetry;

    public enum State {
        STOP, INTAKE, REVERSE, MANUAL, FEEDING, INTAKE_ONLY
    }

    private State currentState = State.STOP;
    private double manualPower = 0.0;
    private boolean isBallDetected = false;
    private ElapsedTime feedTimer = new ElapsedTime();

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        eater1 = hardwareMap.get(DcMotorEx.class, "eater1");
        eater2 = hardwareMap.get(DcMotorEx.class, "eater2");
        
        sweeperLeft = hardwareMap.get(CRServo.class, "sweeperLeft");
        sweeperRight = hardwareMap.get(CRServo.class, "sweeperRight");
        sweeperRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
        colorSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");

        eater1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        eater2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        eater1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        eater2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        stopEater();
    }

    @Override
    public void update() {
        double intakePower = 0;
        double transferPower = 0;
        
        double distanceCm = colorSensor.getDistance(DistanceUnit.CM);
        
        if (distanceCm > Constants.EATER_BALL_DISTANCE_CM + 2.0) {
            isBallDetected = false;
        }

        if (currentState == State.INTAKE && distanceCm < Constants.EATER_BALL_DISTANCE_CM) {
            currentState = State.INTAKE_ONLY;
            isBallDetected = true; 
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

    public boolean isFull() {return false;}

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
