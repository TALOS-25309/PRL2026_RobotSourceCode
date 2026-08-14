package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmode.auto.SubsystemUpdater;
import org.firstinspires.ftc.teamcode.part.Constants;
import org.firstinspires.ftc.teamcode.part.eater.Eater;
import org.firstinspires.ftc.teamcode.part.shooter.Shooter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "1. Auto Test (60cm Shoot)", group = "Test")
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0); // 시작 좌표: (0, 0) 방향 0도
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Eater eater = new Eater();
        Shooter shooter = new Shooter();

        eater.init(hardwareMap, telemetry);
        shooter.init(hardwareMap, telemetry);

        // 텔레옵 부품들을 백그라운드에서 무한 업데이트 해줄 업데이터 생성
        SubsystemUpdater updater = new SubsystemUpdater(eater, shooter);

        // 오토 매크로: 2발 발사 시퀀스 (텔레옵의 스마트 매크로와 동일한 로직을 Action으로 구성)
        Action shootTwoBalls = new SequentialAction(
            shooter.runShooterAction(Constants.FAR_SHOOT_VELOCITY),
            new SleepAction(Constants.SHOOTER_SPOOL_TIME_MS / 1000.0), // 예열 대기 (ms -> s 변환)
            
            eater.feedToShooterAction(),
            new SleepAction(Constants.EATER_FEED_TIME_MS / 1000.0),    // 1발째 급탄
            
            new SleepAction(Constants.RAPID_FIRE_DELAY_FAR_MS / 1000.0), // 연사 딜레이
            
            eater.feedToShooterAction(),
            new SleepAction(Constants.EATER_FEED_TIME_MS / 1000.0),    // 2발째 급탄
            
            shooter.stopShooterAction(),
            eater.stopEaterAction()
        );

        // 첫 번째 이동 경로: 앞으로 24인치 (약 60cm)
        Action driveToShoot = drive.actionBuilder(initialPose)
                .lineToX(24)
                .build();
                
        // 두 번째 이동 경로: 다시 제자리(0)로 뒤로 24인치 복귀
        Action driveBack = drive.actionBuilder(new Pose2d(24, 0, 0))
                .lineToX(0)
                .build();

        telemetry.addLine("Auto Test Ready!");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // 실제 모터에 파워를 인가할 수 있도록 start() 호출
        eater.start();
        shooter.start();

        // 메인 자율주행 시퀀스 (이동 -> 발사 -> 복귀 -> 업데이터 종료)
        Action autoSequence = new SequentialAction(
            driveToShoot,
            shootTwoBalls,
            driveBack,
            new InstantAction(() -> updater.keepRunning = false) // 모든 경로가 끝나면 백그라운드 루프 종료
        );

        // RoadRunner 1.0 실행: 메인 시퀀스와 백그라운드 업데이터를 병렬(Parallel)로 동시 실행
        Actions.runBlocking(
            new ParallelAction(
                autoSequence,
                updater
            )
        );
        
        eater.stop();
        shooter.stop();
    }
}
