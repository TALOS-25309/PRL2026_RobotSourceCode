package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Test: Forward 70", group = "Test")
public class Test_Forward70 extends LinearOpMode {
    @Override
    public void runOpMode() {
        // 로봇의 현재 위치를 (0, 0) 바라보는 방향 0도로 초기화
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        telemetry.addLine("Ready. Press Start to drive forward 70 inches.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // X축 방향으로 정확히 70인치 직진하는 액션 생성
        Action driveAction = drive.actionBuilder(initialPose)
                .lineToX(70)
                .build();

        // 생성한 액션을 실행 (목표 도달까지 대기)
        Actions.runBlocking(driveAction);
        
        telemetry.addLine("Reached 70 inches!");
        telemetry.update();
        sleep(2000);
    }
}
