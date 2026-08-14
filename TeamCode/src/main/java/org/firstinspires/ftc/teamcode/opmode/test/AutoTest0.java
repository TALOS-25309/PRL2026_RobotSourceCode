package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.part.eater.Eater;
import org.firstinspires.ftc.teamcode.part.shooter.Shooter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

/**
 * 사이클 자율주행.
 *
 *   ① 발사 지점 --회전하며--> 벽 앞
 *   ② 제자리에서 회전 (벽과 나란히)
 *   ③ 벽 따라 회전 없이 이동하며 먹기
 *   ④ 인테이크 정지
 *   ⑤ 벽 --회전하며--> 발사 지점 복귀
 *   ⑥ 2발 발사
 *   위를 CYCLE_COUNT회 반복.
 *
 * [중요] 경로를 여러 개의 actionBuilder로 쪼개면, 각 builder에 넘긴 "시작 포즈"가
 * 앞 구간의 실제 끝 포즈와 어긋나면서 로봇이 엉뚱하게 움직인다.
 * 그래서 아래처럼 builder 하나를 계속 이어 붙인다. 이러면 RoadRunner가
 * 끝 포즈를 자동으로 물려받아 어긋날 여지가 없다.
 */
@Autonomous(name = "Auto Test 0", group = "Test")
public class AutoTest0 extends LinearOpMode {

    private final Eater eater = new Eater();
    private final Shooter shooter = new Shooter();

    @Override
    public void runOpMode() {
        // [중요] MecanumDrive에 넘기는 포즈(= localizer 초기 위치)와
        // actionBuilder에 넘기는 포즈(= 경로 시작점)는 반드시 같아야 한다.
        // 다르면 시작하자마자 그 차이가 통째로 위치 오차로 잡혀서
        // HolonomicController 출력이 포화되고 로봇이 엉뚱한 쪽으로 돌진한다.
        Pose2d startPose = new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        TrajectoryActionBuilder builder = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(0,10), 0)
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(0,0),0)
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(10,0),0)
                .waitSeconds(3);

        // 경로 생성은 무거우니 START 대기 전에 미리 끝내둔다.
        Action path = builder.build();

        telemetry.addLine("Auto Test 0 Ready!");
        telemetry.update();

        // [중요] 이게 없으면 runOpMode()가 INIT 단계에서 그대로 실행돼버린다.
        // 경로를 다 돈 뒤 runOpMode()가 리턴하므로 START를 눌러도 아무 일도 안 일어난다.
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(path);
    }
}
