package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.opmode.auto.Constants.*;
import static org.firstinspires.ftc.teamcode.part.Constants.EATER_FEED_TIME_MS;
import static org.firstinspires.ftc.teamcode.part.Constants.FAR_SHOOT_VELOCITY;
import static org.firstinspires.ftc.teamcode.part.Constants.RAPID_FIRE_DELAY_FAR_MS;
import static org.firstinspires.ftc.teamcode.part.Constants.VEL_TOLERANCE;
import static org.firstinspires.ftc.teamcode.part.Constants.VISION_TURN_KP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.feature.Schedule;
import org.firstinspires.ftc.teamcode.feature.TelemetrySystem;
import org.firstinspires.ftc.teamcode.part.Vision;
import org.firstinspires.ftc.teamcode.part.eater.Eater;
import org.firstinspires.ftc.teamcode.part.shooter.Shooter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

/**
 * AutoOp_FarCorner 의 Schedule 기반 버전.
 * <p>
 * 경로 추종만 RoadRunner Action 으로 하고, 순서/타이밍 제어는 전부 Schedule 로 처리합니다.
 * SequentialAction / afterTime / stopAndAdd 대신 Schedule.addTask 와 addConditionalTask 를 씁니다.
 */
@Autonomous(name = "FarZone+Corner (Schedule)", group = "Auto")
public class AutoOp_FarCornerSchedule extends LinearOpMode {

    private final Eater eater = new Eater();
    private final Shooter shooter = new Shooter();
    private final Vision vision = new Vision();
    private MecanumDrive drive;

    private Action path;        // 현재 따라가는 중인 경로 (null 이면 주행 중이 아님)
    private boolean aiming;     // 비전으로 골대 조준 중 (true 면 주행 대신 제자리 회전)
    private final ElapsedTime aimTimer = new ElapsedTime();
    private boolean finished;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, FAR_START_POSE);

        eater.init(hardwareMap, telemetry);
        shooter.init(hardwareMap, telemetry);
        vision.init(hardwareMap, telemetry);
        TelemetrySystem.init(telemetry);
        Schedule.init();

        waitForStart();
        if (isStopRequested()) return;

        eater.start();
        shooter.start();
        vision.start();
        vision.setPipeline(VISION_GOAL_PIPELINE);   // 이 오토는 골대만 보면 된다

        // 시작 지점 -> 발사 지점, 도착하면 조준 후 2발 쏘고 사이클 시작
        drivePath(drive.actionBuilder(FAR_START_POSE)
                .strafeToLinearHeading(FAR_SHOOT_POS, FAR_SHOOT_HEADING)
                .build());
        afterPath(() -> aimAndShoot(() -> cycle(0)));

        while (opModeIsActive() && !finished) {
            TelemetryPacket packet = new TelemetryPacket();

            vision.update();                                    // tx 를 먼저 갱신
            Schedule.update();
            if (path != null && !path.run(packet)) path = null;
            updateAim();
            eater.update();
            shooter.update();

            telemetry.addData("Aiming", aiming);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
        }

        eater.stop();
        shooter.stop();
        vision.stop();
        Schedule.stop();
    }

    /** 인테이크 ON -> 코너 이동 -> 먹는 동안 대기 -> 발사 지점 복귀 -> 2발 발사 -> 다음 사이클 */
    private void cycle(int i) {
        if (i >= CYCLE_COUNT) {
            finish();
            return;
        }

        /** 원래 : INTAKE+TRANSFER --> 하나 먹으면(isDetected = T) : INTAKE **/
        eater.startIntake();    // INTAKE = 인테이크 + 트랜스퍼 (isBallDetected 초기화됨)
        Schedule.addConditionalTask(() -> eater.runIntakeOnly(),
                DELAY_BETWEEN_TWO_INTAKE, () -> eater.isBallDetected());

        // 조준하면서 헤딩이 틀어져 있으므로 실제 현재 위치에서 경로를 만든다
        drivePath(drive.actionBuilder(drive.localizer.getPose())
                .setTangent(EAT_CORNER_TANGENT)
                .strafeToLinearHeading(EAT_CORNER_POS, EAT_CORNER_HEADING)
                .build());


        // 코너 도착 -> EAT_CORNER_WAIT_TIME 동안 먹기 -> 복귀
        afterPath(() -> Schedule.addTask(() -> {
            eater.stop();
            shooter.runShooter(FAR_SHOOT_VELOCITY);   // 복귀하면서 미리 예열
            drivePath(drive.actionBuilder(EAT_CORNER_POSE)
                    .strafeToLinearHeading(FAR_SHOOT_POS, FAR_SHOOT_HEADING)
                    .build());
            afterPath(() -> aimAndShoot(() -> cycle(i + 1)));
        }, EAT_CORNER_WAIT_TIME));
    }

    /** 예열 시작 -> 비전으로 골대 조준 -> 정렬되면 2발 발사 -> next */
    private void aimAndShoot(Runnable next) {
        shooter.runShooter(FAR_SHOOT_VELOCITY);   // 조준하는 동안 예열 (조준 시간이 곧 예열 시간)
        aimAtGoal(() -> shootTwice(next));
    }

    /** 조준 모드 진입. 정렬이 끝나면(또는 타임아웃되면) next 실행 */
    private void aimAtGoal(Runnable next) {
        aiming = true;
        aimTimer.reset();
        Schedule.addConditionalTask(next, Schedule.RUN_INSTANTLY, () -> !aiming);
    }

    /** 조준 중일 때 매 루프 호출: tx 가 0 이 되도록 제자리 회전 (P 제어) */
    private void updateAim() {
        if (!aiming) return;

        drive.updatePoseEstimate();   // 경로 주행 중이 아니라서 아무도 갱신해주지 않는다

        if (aimTimer.seconds() > AIM_TIMEOUT_S) {
            finishAim();              // 못 찾으면 시간 낭비하지 말고 그냥 쏜다
            return;
        }

        if (!vision.hasTarget()) {    // 골대가 안 보이면 제자리에서 천천히 탐색
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), AIM_SEARCH_TURN));
            return;
        }

        double tx = vision.getTx();
        if (Math.abs(tx) < AIM_TOLERANCE_DEG) {
            finishAim();
            return;
        }

        double turn = -tx * VISION_TURN_KP;
        turn = Math.max(-AIM_MAX_TURN, Math.min(AIM_MAX_TURN, turn));                 // 폭주 방지
        if (Math.abs(turn) < AIM_MIN_TURN) turn = Math.signum(turn) * AIM_MIN_TURN;   // 마찰 극복

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), turn));
    }

    private void finishAim() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        aiming = false;
    }

    /** 예열 대기 -> 급탄 -> 연사 딜레이 -> 예열 대기 -> 급탄 -> next */
    private void shootTwice(Runnable next) {
        shooter.runShooter(FAR_SHOOT_VELOCITY);
        whenSpunUp(() -> {
            eater.feedToShooter();
            Schedule.addTask(() -> whenSpunUp(() -> {
                eater.feedToShooter();
                Schedule.addTask(next, EATER_FEED_TIME_MS / 1000.0);
            }), (EATER_FEED_TIME_MS + RAPID_FIRE_DELAY_FAR_MS) / 1000.0);
        });
    }

    /** 플라이휠이 목표 속도에 도달하면(또는 타임아웃되면) 실행 */
    private void whenSpunUp(Runnable task) {
        ElapsedTime timer = new ElapsedTime();
        Schedule.addConditionalTask(task, Schedule.RUN_INSTANTLY, () ->
                Math.abs(shooter.getVelocity() - FAR_SHOOT_VELOCITY) < VEL_TOLERANCE
                        || timer.seconds() > SPINUP_TIMEOUT_S);
    }

    private void finish() {
        shooter.stopShooter();
        eater.stopEater();
        finished = true;
    }

    private void drivePath(Action p) {
        path = p;
    }

    /** 현재 경로 주행이 끝난 뒤에 실행 */
    private void afterPath(Runnable task) {
        Schedule.addConditionalTask(task, Schedule.RUN_INSTANTLY, () -> path == null);
    }
}
