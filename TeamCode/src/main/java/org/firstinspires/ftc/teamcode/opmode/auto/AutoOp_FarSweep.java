package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.*;
import static org.firstinspires.ftc.teamcode.part.Constants.EATER_FEED_TIME_MS;
import static org.firstinspires.ftc.teamcode.part.Constants.FAR_SHOOT_VELOCITY;
import static org.firstinspires.ftc.teamcode.part.Constants.RAPID_FIRE_DELAY_FAR_MS;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.part.eater.Eater;
import org.firstinspires.ftc.teamcode.part.shooter.Shooter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Arrays;

/**
 * FarZone + 벽 훑기(Sweep) 자율주행.
 *
 * AutoOp_FarCorner 와 구조는 똑같고 "먹는 방법"만 다르다.
 *   FarCorner : 코너로 한 번 갔다가 그 자리에서 먹고 그대로 복귀
 *   FarSweep  : 벽에 비스듬히 부딪힌 뒤, 벽에 붙은 채로 게이트/골대 쪽(-x)으로 이동하며 벽을 훑는다
 *
 *   ① 시작 지점 --회전하며--> 발사 지점, 2발 발사
 *   ② 인테이크 ON, 발사 지점 --비스듬히--> 벽 (SWEEP_ENTRY_POS, 코너 쪽)
 *   ③ 자세(SWEEP_HEADING) 유지한 채 벽을 따라 게이트 쪽으로 훑으며 먹기 (SWEEP_END_POS, 저속)
 *   ④ 인테이크 정지, 슈터 예열하며 발사 지점 복귀
 *   ⑤ 2발 발사
 *   ②~⑤ 를 CYCLE_COUNT 회 반복.
 *
 * [중요] 경로를 여러 actionBuilder로 쪼개면 각 builder에 넘긴 시작 포즈가 앞 구간의 실제 끝
 * 포즈와 어긋난다. FarCorner 와 마찬가지로 builder 하나를 계속 이어 붙인다.
 */
@Autonomous(name = "FarZone+Sweep", group = "Auto")
public class AutoOp_FarSweep extends LinearOpMode {

    private final Eater eater = new Eater();
    private final Shooter shooter = new Shooter();

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, FAR_START_POSE);

        eater.init(hardwareMap, telemetry);
        shooter.init(hardwareMap, telemetry);

        SubsystemUpdater updater = new SubsystemUpdater(eater, shooter);

        // 훑는 구간은 기본 속도(50 in/s)로 가면 공이 튕겨나가므로 느리게 간다.
        VelConstraint sweepVel = new MinVelConstraint(Arrays.asList(
                drive.kinematics.new WheelVelConstraint(EAT_MAX_VEL),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint sweepAccel = new ProfileAccelConstraint(-EAT_MAX_ACCEL, EAT_MAX_ACCEL);

        TrajectoryActionBuilder builder = drive.actionBuilder(FAR_START_POSE)
                .strafeToLinearHeading(FAR_SHOOT_POS, FAR_SHOOT_HEADING)
                .stopAndAdd(shootTwice());

        for (int i = 0; i < CYCLE_COUNT; i++) {
            builder = builder
                    // ② 인테이크 ON 하고 벽으로 비스듬히 들이받기
                    //    (진입 자세를 이미 훑기 자세로 잡아두면 벽 앞에서 따로 회전할 필요가 없다)
                    .stopAndAdd(eater.startIntakeAction())
                    .setTangent(SWEEP_ENTRY_TANGENT)
                    .strafeToLinearHeading(SWEEP_ENTRY_POS, SWEEP_HEADING)

                    // ③ 벽에 붙은 채로 게이트 쪽(-x)으로 훑으며 먹기 (자세 유지, 저속)
                    //    setTangent 가 여기서 경로를 한 번 끊어주므로, 벽에 부딪혀 멈춘 뒤 훑기가 시작된다.
                    //    strafeTo 는 헤딩이 진행 방향을 따라가버리므로 반드시 ConstantHeading 을 쓴다.
                    .setTangent(SWEEP_TANGENT)
                    .strafeToConstantHeading(SWEEP_END_POS, sweepVel, sweepAccel)

                    // ④ 마지막 공까지 들어갈 시간을 조금 준 뒤 인테이크 정지, 복귀하면서 예열
                    .afterTime(SWEEP_TAIL_TIME, eater.stopEaterAction())
                    .afterTime(0.5, shooter.runShooterAction(FAR_SHOOT_VELOCITY))
                    .strafeToLinearHeading(FAR_SHOOT_POS, FAR_SHOOT_HEADING)

                    // ⑤ 2발 발사
                    .stopAndAdd(shootTwice());
        }

        Action autoSequence = new SequentialAction(
                builder.build(),
                shooter.stopShooterAction(),
                eater.stopEaterAction(),
                new InstantAction(() -> updater.keepRunning = false)
        );

        waitForStart();
        if (isStopRequested()) return;

        eater.start();
        shooter.start();

        Actions.runBlocking(
                new ParallelAction(
                        autoSequence,
                        updater
                )
        );

        eater.stop();
        shooter.stop();
    }

    /** 목표 속도 도달 대기 -> 급탄 -> 속도 회복 대기 -> 급탄. */
    private Action shootTwice() {
        return new SequentialAction(
                shooter.waitUntilTargetVelocityAction(FAR_SHOOT_VELOCITY, SPINUP_TIMEOUT_S),
                eater.feedToShooterAction(),
                new SleepAction(EATER_FEED_TIME_MS / 1000.0),        // 1발째 급탄 완료 대기
                new SleepAction(RAPID_FIRE_DELAY_FAR_MS / 1000.0),   // 연사 딜레이

                shooter.waitUntilTargetVelocityAction(FAR_SHOOT_VELOCITY, SPINUP_TIMEOUT_S),
                eater.feedToShooterAction(),
                new SleepAction(EATER_FEED_TIME_MS / 1000.0)         // 2발째 급탄 완료 대기
        );
    }
}
