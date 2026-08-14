package org.firstinspires.ftc.teamcode.part.shooter;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.part.Constants;
import org.firstinspires.ftc.teamcode.part.Part;
import static org.firstinspires.ftc.teamcode.part.Constants.*;

public class Shooter implements Part {

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private Telemetry telemetry;

    public enum MotorState {
        STOP, RUN, MANUAL
    }

    private MotorState motorState = MotorState.STOP;
    private double manualPower = 0.0;
    private double currentTargetVelocity = 0.0;
    
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void start() {
        stopShooter();
    }

    @Override
    public void update() {
        switch (motorState) {
            case RUN:
                shooter1.setVelocity(currentTargetVelocity);
                shooter2.setVelocity(currentTargetVelocity);
                break;
            case MANUAL:
                shooter1.setPower(manualPower);
                shooter2.setPower(manualPower);
                break;
            case STOP:
            default:
                shooter1.setVelocity(0);
                shooter2.setVelocity(0);
                break;
        }

        telemetry.addData("Shooter State", motorState);
        telemetry.addData("Target Vel", motorState == MotorState.RUN ? currentTargetVelocity : 0);
        telemetry.addData("Actual Vel 1", shooter1.getVelocity());
        telemetry.addData("Actual Vel 2", shooter2.getVelocity());
    }

    @Override
    public void stop() {
        stopShooter();
    }

    public void runShooter(double targetVelocity) {
        motorState = MotorState.RUN;
        currentTargetVelocity = targetVelocity;
    }

    public void stopShooter() {
        motorState = MotorState.STOP;
    }

    public double getVelocity() {
        return shooter1.getVelocity();
    }

    public void manualShooter(double power) {
        motorState = MotorState.MANUAL;
        this.manualPower = power;
    }
    
    public Action runShooterAction(double targetVelocity) {
        return new InstantAction(() -> runShooter(targetVelocity));
    }
    
    public Action stopShooterAction() {
        return new InstantAction(() -> stopShooter());
    }

    public Action waitUntilTargetVelocityAction(double targetVelocity, double maxTime) {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            boolean init=true;
            double curVelocity;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(init){
                    runShooter(targetVelocity);
                    timer.reset();
                    init = false;
                }
                if(timer.seconds() > maxTime) return false;
                curVelocity = getVelocity();
                return Math.abs(curVelocity - targetVelocity) > VEL_TOLERANCE;
            }
        };
    }
}
