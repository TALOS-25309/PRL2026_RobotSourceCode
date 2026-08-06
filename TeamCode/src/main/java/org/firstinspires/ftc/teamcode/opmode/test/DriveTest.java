//package org.firstinspires.ftc.teamcode.opmode.test;
//
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.part.Drive;
//
//@TeleOp(name = "Drive Test", group = "Test")
//public class DriveTest extends LinearOpMode {
//
//    private Drive drive;
//
//    @Override
//    public void runOpMode() {
//        drive = new Drive();
//
//        // 하드웨어 맵 매핑 및 RoadRunner 초기화
//        drive.init(hardwareMap, telemetry);
//
//        telemetry.addLine("Drive Test Ready!");
//        telemetry.addLine("Left Stick Y: Forward/Backward");
//        telemetry.addLine("Left Stick X: Strafe Left/Right");
//        telemetry.addLine("Right Stick X: Turn Left/Right");
//        telemetry.update();
//
//        waitForStart();
//
//        drive.start();
//
//        while (opModeIsActive()) {
//
//            // 조이스틱 입력 받기
//            // Y축은 위로 밀 때 음수값이 나오므로 반전(-)시켜서 전진(양수)으로 만듭니다.
//            // RoadRunner 1.0에서는 로봇 앞쪽이 X축, 왼쪽이 Y축 양수 방향입니다.
//            double x = -gamepad1.left_stick_y;
//            double y = -gamepad1.left_stick_x;
//            double rx = -gamepad1.right_stick_x; // 모터 방향이 정상화되었으므로 다시 원래 부호(-)로 복구
//
//            // 조이스틱 데드존 설정 (약한 떨림 무시)
//            if (Math.abs(x) < 0.05) x = 0;
//            if (Math.abs(y) < 0.05) y = 0;
//            if (Math.abs(rx) < 0.05) rx = 0;
//
//            // 주행 파워 설정 (PoseVelocity2d 객체로 묶어서 전달)
//            drive.setDrivePower(new PoseVelocity2d(
//                new Vector2d(x, y),
//                rx
//            ));
//
//            // 시스템 업데이트 (모터 구동 및 Odometry 위치 추적 수행)
//            drive.update();
//            telemetry.update();
//        }
//
//        drive.stop();
//    }
//}
