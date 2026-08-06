package org.firstinspires.ftc.teamcode.opmode.test;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.part.eater.Eater;
//
//@TeleOp(name = "Eater Test", group = "Test")
//public class EaterTest extends LinearOpMode {
//
//    private Eater eater;
//    private boolean lastBallDetected = false;
//
//    @Override
//    public void runOpMode() {
//        eater = new Eater();
//        eater.init(hardwareMap, telemetry);
//
//        telemetry.addLine("Eater Test Ready!");
//        telemetry.addLine("Hold 'Right Trigger' to Intake");
//        telemetry.addLine("Hold 'Left Trigger' to Reverse");
//        telemetry.addLine("※ Auto-stop is enabled based on motor current.");
//        telemetry.update();
//
//        waitForStart();
//
//        eater.start();
//
//        while (opModeIsActive()) {
//
//            // 1. 트리거로 인테이크 조작
//            if (gamepad1.right_trigger > 0.1) {
//                eater.runIntake();
//            } else if (gamepad1.left_trigger > 0.1) {
//                eater.runReverse();
//            } else {
//                eater.stopEater();
//            }
//
//            // 2. 부품 업데이트 (여기서 전류 검사 및 자동 정지가 일어남)
//            eater.update();
//
//            // 3. 공 감지 시 조종기 진동 (Rumble) 손맛 피드백
//            boolean currentBallDetected = eater.isBallDetected();
//            if (currentBallDetected && !lastBallDetected) {
//                // 방금 막 공이 감지되어 모터가 자동 정지된 순간
//                gamepad1.rumble(500); // 500ms(0.5초) 동안 조종기 1번 진동
//                gamepad2.rumble(500); // 조종기 2번도 같이 진동
//            }
//            lastBallDetected = currentBallDetected;
//
//            telemetry.update();
//        }
//
//        eater.stop();
//    }
//}
