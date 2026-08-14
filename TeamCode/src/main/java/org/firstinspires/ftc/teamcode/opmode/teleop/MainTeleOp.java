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

    private enum ShootMacroState {
        IDLE,
        SPOOLING,
        FEEDING,
        WAITING_FOR_NEXT_RAPID
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

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        drive.start();
        eater.start();
        shooter.start();
        vision.start();

        while (opModeIsActive()) {
            
            double forward = -gamepad1.left_stick_y * Constants.DRIVE_SPEED_MULTIPLIER;
            double strafe = -gamepad1.left_stick_x * Constants.DRIVE_SPEED_MULTIPLIER;
            double turn = -gamepad1.right_stick_x * Constants.DRIVE_SPEED_MULTIPLIER;

            if (gamepad1.x && vision.hasTarget()) {
                turn = -vision.getTx() * Constants.VISION_TURN_KP;
            }
            
            drive.setDrivePower(new PoseVelocity2d(new Vector2d(forward, strafe), turn));

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

            boolean reqFarSingle = gamepad1.y;
            boolean reqFarRapid = gamepad1.right_trigger > 0.1;
            boolean reqCloseSingle = gamepad1.a;
            boolean reqCloseRapid = gamepad1.right_bumper;

            boolean triggerPressed = reqFarSingle || reqFarRapid || reqCloseSingle || reqCloseRapid;
            boolean holdingRapid = reqFarRapid || reqCloseRapid;

            switch (shootState) {
                case IDLE:
                    if (triggerPressed) {
                        if (reqFarSingle || reqFarRapid) {
                            currentMacroVelocity = Constants.FAR_SHOOT_VELOCITY;
                            currentMacroRapidDelay = Constants.RAPID_FIRE_DELAY_FAR_MS;
                        } else {
                            currentMacroVelocity = Constants.CLOSE_SHOOT_VELOCITY;
                            currentMacroRapidDelay = Constants.RAPID_FIRE_DELAY_CLOSE_MS;
                        }
                        isRapidMode = reqFarRapid || reqCloseRapid;
                        
                        shooter.runShooter(currentMacroVelocity);
                        macroTimer.reset();
                        shootState = ShootMacroState.SPOOLING;
                    } else {
                        shooter.stopShooter();
                    }
                    break;

                case SPOOLING:
                    if (isRapidMode && !holdingRapid) {
                        shootState = ShootMacroState.IDLE;
                        break;
                    }
                    
                    if (macroTimer.milliseconds() >= Constants.SHOOTER_SPOOL_TIME_MS) {
                        eater.feedToShooter();
                        macroTimer.reset();
                        shootState = ShootMacroState.FEEDING;
                    }
                    break;

                case FEEDING:
                    if (isRapidMode && !holdingRapid) {
                        isRapidMode = false;
                    }

                    if (macroTimer.milliseconds() >= Constants.EATER_FEED_TIME_MS) {
                        if (isRapidMode) {
                            macroTimer.reset();
                            shootState = ShootMacroState.WAITING_FOR_NEXT_RAPID;
                        } else {
                            shootState = ShootMacroState.IDLE;
                        }
                    }
                    break;

                case WAITING_FOR_NEXT_RAPID:
                    if (!holdingRapid) {
                        shootState = ShootMacroState.IDLE;
                        break;
                    }
                    
                    if (macroTimer.milliseconds() >= currentMacroRapidDelay) {
                        eater.feedToShooter();
                        macroTimer.reset();
                        shootState = ShootMacroState.FEEDING;
                    }
                    break;
            }

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
