//package org.firstinspires.ftc.teamcode.opmode.test;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.part.eater.Eater;
//
//@TeleOp(name = "Stopper Calibration Test", group = "Test")
//public class StopperCalibrationTest extends LinearOpMode {
//
//    private Servo stopperL;
//    private Servo stopperR;
//
//    // 트랜스퍼 테스트용 Eater 부품 추가
//    private Eater eater;
//
//    // 서보 모터의 초기 테스트 각도
//    private double posL = 0.5;
//    private double posR = 0.5;
//
//    @Override
//    public void runOpMode() {
//        stopperL = hardwareMap.get(Servo.class, "stopperL");
//        stopperR = hardwareMap.get(Servo.class, "stopperR");
//
//        eater = new Eater();
//        eater.init(hardwareMap, telemetry);
//
//        telemetry.addLine("Stopper Calibration Ready");
//        telemetry.addLine("DPAD UP/DOWN: Left Stopper (stopperL)");
//        telemetry.addLine("DPAD LEFT/RIGHT: Right Stopper (stopperR)");
//        telemetry.addLine("[RB] / [LB]: Move BOTH Stoppers Together (+ / -)");
//        telemetry.addLine("Hold 'Right Trigger' (RT): Spin Transfer/Eater Motors");
//        telemetry.addLine("Press START when ready.");
//        telemetry.update();
//
//        waitForStart();
//
//        eater.start();
//
//        while (opModeIsActive()) {
//
//            // 1. 왼쪽 스토퍼 미세 조절 (0.001 단위)
//            if (gamepad1.dpad_up) {
//                posL += 0.001;
//            } else if (gamepad1.dpad_down) {
//                posL -= 0.001;
//            }
//
//            // 2. 오른쪽 스토퍼 미세 조절 (0.001 단위)
//            if (gamepad1.dpad_right) {
//                posR += 0.001;
//            } else if (gamepad1.dpad_left) {
//                posR -= 0.001;
//            }
//
//            // 2.5 양쪽 스토퍼 동시 미세 조절 (0.001 단위)
//            if (gamepad1.right_bumper) {
//                posL += 0.001;
//                posR += 0.001;
//            } else if (gamepad1.left_bumper) {
//                posL -= 0.001;
//                posR -= 0.001;
//            }
//
//            // 값 범위 제한 (0.0 ~ 1.0)
//            if (posL > 1.0) posL = 1.0;
//            if (posL < 0.0) posL = 0.0;
//            if (posR > 1.0) posR = 1.0;
//            if (posR < 0.0) posR = 0.0;
//
//            // 서보에 직접 값 인가
//            stopperL.setPosition(posL);
//            stopperR.setPosition(posR);
//
//            // 3. 트랜스퍼 가동 테스트
//            if (gamepad1.right_trigger > 0.1) {
//                eater.runIntake(); // RT를 누르면 트랜스퍼가 돌며 공을 밀어올림
//            } else {
//                eater.stopEater();
//            }
//            eater.update();
//
//            // 화면 출력
//            telemetry.addLine("--- Stopper Calibration ---");
//            telemetry.addData("Left Stopper Position", "%.3f", posL);
//            telemetry.addData("Right Stopper Position", "%.3f", posR);
//            telemetry.addLine("\n[튜닝 방법]");
//            telemetry.addLine("1. 십자키 또는 LB/RB 버튼으로 스토퍼 각도를 조절합니다.");
//            telemetry.addLine("2. RT를 꾹 눌러 트랜스퍼를 돌리며 공이 잘 통과하는지(혹은 잘 막히는지) 테스트합니다.");
//            telemetry.addLine("3. 찾아낸 완벽한 값을 Constants.java 에 적어주세요!");
//            telemetry.update();
//        }
//
//        eater.stop();
//    }
//}
