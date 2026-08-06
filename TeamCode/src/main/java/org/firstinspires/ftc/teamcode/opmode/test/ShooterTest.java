//package org.firstinspires.ftc.teamcode.opmode.test;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.part.shooter.Shooter;
//
//@TeleOp(name = "Shooter Test", group = "Test")
//public class ShooterTest extends LinearOpMode {
//
//    private Shooter shooter;
//    private boolean lastAPressed = false;
//
//    @Override
//    public void runOpMode() {
//        shooter = new Shooter();
//        shooter.init(hardwareMap, telemetry);
//
//        telemetry.addLine("Shooter Test Ready!");
//        telemetry.addLine("Hold 'Right Trigger' to spin up Shooter (PID Velocity Control)");
//        telemetry.addLine("Use 'Right Stick Y' for Manual Speed Control");
//        telemetry.addLine("Press 'A' to Auto Fire (Opens stoppzer for 0.3s)");
//        telemetry.addLine("※ Check FTC Dashboard for Velocity graphs!");
//        telemetry.update();
//
//        waitForStart();
//
//        shooter.start();
//
//        while (opModeIsActive()) {
//
//            // 1. 슈터 플라이휠 (모터 1, 2) 제어
//            double rightStickY = -gamepad1.right_stick_y; // 조이스틱 위로 올리면 양수
//
//            if (Math.abs(rightStickY) > 0.05) {
//                // 오른쪽 조이스틱으로 미세 속도 조절 (Manual)
//                shooter.manualShooter(rightStickY);
//            } else if (gamepad1.right_trigger > 0.1) {
//                // 오른쪽 트리거(RT)를 누르면 상수(Constants)에 지정된 타겟 RPM(Velocity)으로 회전
//                shooter.runShooter();
//            } else {
//                shooter.stopShooter();
//            }
//
//            // 2. 스마트 자동 발사 시퀀스 (A 버튼)
//            boolean aPressed = gamepad1.a;
//            if (aPressed && !lastAPressed) {
//                // A 버튼을 '누르는 순간' 단 1번만 fire() 호출 (스토퍼 자동 개폐)
//                shooter.fire();
//            }
//            lastAPressed = aPressed;
//
//            // 3. 업데이트 및 상태 출력
//            // 속도 제어 데이터가 Telemetry로 출력되므로, FTC Dashboard에서 그래프로 확인할 수 있습니다.
//            shooter.update();
//            telemetry.update();
//        }
//
//        shooter.stop();
//    }
//}
