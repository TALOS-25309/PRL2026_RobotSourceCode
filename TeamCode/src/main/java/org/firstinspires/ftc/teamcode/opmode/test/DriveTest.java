package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.part.Drive;

@TeleOp(name = "Drive Test", group = "Test")
public class DriveTest extends LinearOpMode {

    // 버튼 주행 파워 (한 축만 순수하게 인가)
    private static final double BUTTON_POWER = 0.4;

    private Drive drive;

    // 기준 자세 (버튼 주행 시작 시점 / A 버튼 리셋 시점)
    private Pose2d refPose = new Pose2d(0, 0, 0);
    private boolean wasButtonMove = false;

    @Override
    public void runOpMode() {
        drive = new Drive();

        // 하드웨어 맵 매핑 및 RoadRunner 초기화
        drive.init(hardwareMap, telemetry);

        telemetry.addLine("Drive Test Ready!");
        telemetry.addLine("Stick: Forward / Strafe / Turn");
        telemetry.addLine("DPad U-D: Pure Forward/Back");
        telemetry.addLine("DPad L-R: Pure Strafe");
        telemetry.addLine("Bumper L-R: Pure Turn");
        telemetry.addLine("A: Reset reference (error zero)");
        telemetry.update();

        waitForStart();

        drive.start();

        while (opModeIsActive()) {

            // 조이스틱 입력 받기
            // Y축은 위로 밀 때 음수값이 나오므로 반전(-)시켜서 전진(양수)으로 만듭니다.
            // RoadRunner 1.0에서는 로봇 앞쪽이 X축, 왼쪽이 Y축 양수 방향입니다.
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            // 조이스틱 데드존 설정 (약한 떨림 무시)
            if (Math.abs(x) < 0.05) x = 0;
            if (Math.abs(y) < 0.05) y = 0;
            if (Math.abs(rx) < 0.05) rx = 0;

            // 버튼 주행: 한 축만 인가하고 나머지는 0으로 강제 (축 간섭 제거)
            String mode = "STICK";
            boolean buttonMove = true;
            if (gamepad1.dpad_up) {
                x = BUTTON_POWER;  y = 0; rx = 0; mode = "FORWARD";
            } else if (gamepad1.dpad_down) {
                x = -BUTTON_POWER; y = 0; rx = 0; mode = "BACK";
            } else if (gamepad1.dpad_left) {
                x = 0; y = BUTTON_POWER;  rx = 0; mode = "STRAFE L";
            } else if (gamepad1.dpad_right) {
                x = 0; y = -BUTTON_POWER; rx = 0; mode = "STRAFE R";
            } else if (gamepad1.left_bumper) {
                x = 0; y = 0; rx = BUTTON_POWER;  mode = "TURN L";
            } else if (gamepad1.right_bumper) {
                x = 0; y = 0; rx = -BUTTON_POWER; mode = "TURN R";
            } else {
                buttonMove = false;
            }

            Pose2d pose = drive.mecanumDrive.localizer.getPose();

            // 버튼 주행이 새로 시작되는 순간 또는 A 버튼을 누르면 기준 자세를 갱신
            if ((buttonMove && !wasButtonMove) || gamepad1.a) {
                refPose = pose;
            }
            wasButtonMove = buttonMove;

            // 주행 파워 설정 (PoseVelocity2d 객체로 묶어서 전달)
            drive.setDrivePower(new PoseVelocity2d(
                new Vector2d(x, y),
                rx
            ));

            // 기준 자세 대비 이동/오차 (기준 시점의 로봇 좌표계 기준)
            double h0 = refPose.heading.toDouble();
            double dx = pose.position.x - refPose.position.x;
            double dy = pose.position.y - refPose.position.y;
            double fwd = dx * Math.cos(h0) + dy * Math.sin(h0);
            double lat = -dx * Math.sin(h0) + dy * Math.cos(h0);
            double dh = normalizeAngle(pose.heading.toDouble() - h0);

            // 시스템 업데이트 (모터 구동 및 Odometry 위치 추적 수행)
            drive.update();

            telemetry.addData("Mode", mode);
            telemetry.addData("Power (x/y/rx)", "%.2f / %.2f / %.2f", x, y, rx);
            telemetry.addLine();
            telemetry.addData("d Forward (in)", "%.2f", fwd);
            telemetry.addData("d Lateral (in)", "%.2f", lat);
            telemetry.addData("d Heading (deg)", "%.2f", Math.toDegrees(dh));
            telemetry.update();
        }

        drive.stop();
    }

    /** 각도를 -180~180도(라디안) 범위로 정규화 */
    private double normalizeAngle(double rad) {
        while (rad > Math.PI) rad -= 2 * Math.PI;
        while (rad < -Math.PI) rad += 2 * Math.PI;
        return rad;
    }
}
