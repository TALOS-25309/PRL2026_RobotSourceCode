package org.firstinspires.ftc.teamcode.opmode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.part.Constants;
import org.firstinspires.ftc.teamcode.part.Vision;
import org.firstinspires.ftc.teamcode.part.eater.Eater;
import org.firstinspires.ftc.teamcode.part.shooter.Shooter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Zone 1 (1m 30s Sweep & Shoot)", group = "Auto")
public class AutoZone1 extends LinearOpMode {

    // --- 커스텀 Action: 라임라이트 오토 에임 ---
    public class AutoAimAction implements Action {
        private final MecanumDrive drive;
        private final Vision vision;

        public AutoAimAction(MecanumDrive drive, Vision vision) {
            this.drive = drive;
            this.vision = vision;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            vision.update(); // 라임라이트 최신 데이터 갱신
            if (vision.hasTarget()) {
                double tx = vision.getTx();
                // 1.5도 이내로 오차가 줄어들면 정렬 완료
                if (Math.abs(tx) < 1.5) {
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    return false; // Action 종료 (다음 단계로 넘어감)
                }

                double turnPower = -tx * Constants.VISION_TURN_KP;
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), turnPower));
                drive.updatePoseEstimate();
                return true; // 계속 정렬 중
            } else {
                // 타겟이 없으면 일단 정지 (타겟을 못 찾았더라도 일단 쏘고 넘어감)
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                return false;
            }
        }
    }

    @Override
    public void runOpMode() {
        // 시작 좌표: (0, 0) 방향 0도 (골대를 바라보는 방향)
        Pose2d currentPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, currentPose);
        Eater eater = new Eater();
        Shooter shooter = new Shooter();
        Vision vision = new Vision();

        eater.init(hardwareMap, telemetry);
        shooter.init(hardwareMap, telemetry);
        vision.init(hardwareMap, telemetry);

        SubsystemUpdater updater = new SubsystemUpdater(eater, shooter, vision);

        telemetry.addLine("Zone 1 Auto Ready!");
        telemetry.addLine("70x70 Quadrant Sweep & Limelight AutoAim");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        eater.start();
        shooter.start();
        vision.start();

        // 2발 발사 시퀀스 (오토 에임 직후 실행됨)
        Action shootTwoBalls = new SequentialAction(
                shooter.runShooterAction(Constants.FAR_SHOOT_VELOCITY),
                new SleepAction(Constants.SHOOTER_SPOOL_TIME_MS / 1000.0), // 예열 대기

                eater.feedToShooterAction(),
                new SleepAction(Constants.EATER_FEED_TIME_MS / 1000.0),    // 1발째 급탄

                new SleepAction(Constants.RAPID_FIRE_DELAY_FAR_MS / 1000.0), // 연사 딜레이

                eater.feedToShooterAction(),
                new SleepAction(Constants.EATER_FEED_TIME_MS / 1000.0),    // 2발째 급탄

                shooter.stopShooterAction()
        );

        // --- 1분 30초 Sweep(지그재그 탐색) 시퀀스 빌드 ---
        // 각 지점으로 이동하며 바닥의 공을 주워담고(Intake ON), 지점에 도착하면 골대를 찾아(AutoAim) 발사합니다.
        Action sweepAndShootSequence = new SequentialAction(
                // 1. (0,0) -> (30,0)
                eater.runIntakeAction(),
                drive.actionBuilder(currentPose).lineToX(30).turnTo(0).build(),
                eater.stopEaterAction(),
                new AutoAimAction(drive, vision),
                shootTwoBalls,

                // 2. (30,0) -> (30,30)
                eater.runIntakeAction(),
                drive.actionBuilder(new Pose2d(30, 0, 0)).lineToY(30).turnTo(0).build(),
                eater.stopEaterAction(),
                new AutoAimAction(drive, vision),
                shootTwoBalls,

                // 3. (30,30) -> (60,30)
                eater.runIntakeAction(),
                drive.actionBuilder(new Pose2d(30, 30, 0)).lineToX(60).turnTo(0).build(),
                eater.stopEaterAction(),
                new AutoAimAction(drive, vision),
                shootTwoBalls,

                // 4. (60,30) -> (60,60)
                eater.runIntakeAction(),
                drive.actionBuilder(new Pose2d(60, 30, 0)).lineToY(60).turnTo(0).build(),
                eater.stopEaterAction(),
                new AutoAimAction(drive, vision),
                shootTwoBalls,

                // 5. (60,60) -> (30,60)
                eater.runIntakeAction(),
                drive.actionBuilder(new Pose2d(60, 60, 0)).lineToX(30).turnTo(0).build(),
                eater.stopEaterAction(),
                new AutoAimAction(drive, vision),
                shootTwoBalls,
                
                // 6. 시작점으로 복귀 및 종료
                drive.actionBuilder(new Pose2d(30, 60, 0)).lineToY(0).lineToX(0).turnTo(0).build(),
                new InstantAction(() -> updater.keepRunning = false)
        );

        // 메인 시퀀스와 백그라운드 업데이터 병렬 실행
        Actions.runBlocking(
                new ParallelAction(
                        sweepAndShootSequence,
                        updater
                )
        );

        eater.stop();
        shooter.stop();
        vision.stop();
    }
}
