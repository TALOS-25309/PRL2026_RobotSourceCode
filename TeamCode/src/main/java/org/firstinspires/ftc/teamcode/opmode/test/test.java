package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * 인테이크 / 트랜스퍼 / 슈터 통합 튜닝 OpMode.
 *
 * [특징]
 *  1. Eater/Shooter 클래스를 거치지 않고 하드웨어를 직접 잡습니다.
 *     -> 인테이크(eater1), 트랜스퍼(eater2), 스위퍼(CRServo), 슈터(shooter1/2)를 각각 따로 돌릴 수 있습니다.
 *  2. 모든 상수를 FTC Dashboard(TUNE 그룹)와 조종기 양쪽에서 실시간으로 조정할 수 있습니다.
 *     -> 노트북이 없어도 조종기만으로 값 조정이 가능합니다.
 *  3. 예열 실측 시간 / 급탄 후 속도 회복 시간을 측정해서 보여줍니다.
 *     -> SHOOTER_SPOOL_TIME_MS, RAPID_FIRE_DELAY_* 값을 감이 아니라 실측으로 정할 수 있습니다.
 *
 * [주의] 여기서 조정한 값은 이 OpMode 안에서만 유효합니다.
 *        튜닝이 끝나면 각 항목에 표시된 위치(part/Constants.java 등)에 손으로 옮겨 적어야 합니다.
 */
@Config("TUNE")
@TeleOp(name = "0. Tune (Intake/Transfer/Shooter)", group = "Test")
public class test extends LinearOpMode {

    // ==========================================================
    //  대시보드 설정값 (TUNE > INTAKE / TRANSFER / SHOOTER)
    // ==========================================================

    public static IntakeCfg INTAKE = new IntakeCfg();
    public static TransferCfg TRANSFER = new TransferCfg();
    public static ShooterCfg SHOOTER = new ShooterCfg();

    /** 인테이크(eater1) + 스위퍼(CRServo) 관련 상수 */
    public static class IntakeCfg {
        public double power = 1.0;                  // 정방향 파워
        public double reversePower = -1.0;          // 역방향(뱉기) 파워
        public double sweeperPower = 1.0;           // 스위퍼 정방향 파워
        public double sweeperReversePower = -1.0;   // 스위퍼 역방향 파워
        public boolean sweeperFollowIntake = true;  // 스위퍼를 인테이크와 함께 돌릴지
        public double ballDistanceCm = 4.5;         // 이 거리보다 가까우면 공 감지
        public double ballClearMarginCm = 2.0;      // 감지 해제 여유 (감지거리 + 이 값 초과 시 해제)
        public boolean motorReverse = false;        // eater1 방향 반전
        public boolean sweeperLeftReverse = false;  // sweeperLeft 방향 반전
        public boolean sweeperRightReverse = true;  // sweeperRight 방향 반전 (Eater.java 기본값과 동일)
    }

    /** 트랜스퍼(eater2) 관련 상수 */
    public static class TransferCfg {
        public double power = 1.0;                  // 정방향 파워
        public double reversePower = -1.0;          // 역방향 파워
        public double feedPower = 1.0;              // 급탄(발사) 시 파워
        public double feedTimeMs = 350;             // 급탄 1회 지속 시간
        public boolean feedRunsIntake = true;       // 급탄 중 인테이크도 같이 돌릴지
        public boolean motorReverse = false;        // eater2 방향 반전
    }

    /** 슈터(shooter1/2) 관련 상수 */
    public static class ShooterCfg {
        public double farVelocity = 2400.0;         // 파슛 목표 속도 (ticks/sec)
        public double closeVelocity = 2100.0;       // 골밑슛 목표 속도 (ticks/sec)
        public double idleVelocity = 0.0;           // 대기 중 유지 속도 (0이면 완전 정지)
        public double manualPower = 0.7;            // 수동 모드 파워
        public double velTolerance = 50;            // 목표 도달 판정 허용 오차
        public double spoolTimeMs = 3000;           // 발사 전 예열 시간
        public boolean spoolWaitForAtSpeed = false; // ON이면 속도 도달 시 즉시 발사(예열 시간은 타임아웃으로만 사용)
        public double rapidDelayFarMs = 700;        // 파슛 연사 간격
        public double rapidDelayCloseMs = 370;      // 골밑슛 연사 간격
        public boolean usePidf = false;             // 커스텀 속도 PIDF 사용 여부
        public double kP = 1.2;
        public double kI = 0.12;
        public double kD = 0.0;
        public double kF = 11.7;                    // 32767 / 최대속도(ticks/sec) 근사값
        public boolean motor1Reverse = true;        // Shooter.java 기본값과 동일
        public boolean motor2Reverse = false;
    }

    // ==========================================================
    //  하드웨어
    // ==========================================================

    private DcMotorEx intakeMotor;      // eater1
    private DcMotorEx transferMotor;    // eater2
    private CRServo sweeperLeft;
    private CRServo sweeperRight;
    private DistanceSensor ballSensor;  // colorSensor
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;

    private final List<String> missingDevices = new ArrayList<>();
    private PIDFCoefficients stockPidf = null;  // 모터 기본 PIDF (usePidf를 끄면 이 값으로 복구)
    private String pidfError = null;

    // ==========================================================
    //  실행 상태
    // ==========================================================

    private enum ShootMode { STOP, FAR, CLOSE, MANUAL }
    private enum Macro { OFF, SPOOL, FEED, WAIT }

    private ShootMode shootMode = ShootMode.STOP;
    private Macro macro = Macro.OFF;

    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime macroTimer = new ElapsedTime();
    private final ElapsedTime spinupTimer = new ElapsedTime();
    private final ElapsedTime recoveryTimer = new ElapsedTime();

    private boolean feeding = false;
    private boolean ballDetected = false;
    private boolean showHelp = true;

    // 실측값 (튜닝용)
    private double lastTarget = 0;
    private boolean spinupMeasured = true;
    private double lastSpinupMs = -1;       // 목표 속도까지 걸린 실제 시간
    private boolean recovering = false;
    private double lastRecoveryMs = -1;     // 급탄 후 속도 회복까지 걸린 시간
    private double minVelAfterFeed = -1;    // 급탄 직후 최저 속도 (속도 낙폭)

    // 적용된 PIDF 추적 (값이 바뀔 때만 허브로 전송)
    private double appliedP = Double.NaN, appliedI = Double.NaN, appliedD = Double.NaN, appliedF = Double.NaN;

    // 현재 인가 중인 출력 (텔레메트리용)
    private double intakePower = 0, transferPower = 0, sweeperPower = 0;
    private double targetVelocity = 0;
    private boolean atSpeed = false;

    // ==========================================================
    //  튜닝 에디터
    // ==========================================================

    private interface Getter { double get(); }
    private interface Setter { void set(double v); }

    /** 조종기로 조정 가능한 값 하나 */
    private static class Param {
        final String name;      // 표시 이름
        final String target;    // 실제 코드에서 이 값이 있는 위치
        final double step, min, max;
        final boolean toggle;   // true면 ON/OFF로 표시
        final Getter getter;
        final Setter setter;
        final double defaultValue;

        Param(String name, String target, double step, double min, double max,
              boolean toggle, Getter getter, Setter setter) {
            this.name = name;
            this.target = target;
            this.step = step;
            this.min = min;
            this.max = max;
            this.toggle = toggle;
            this.getter = getter;
            this.setter = setter;
            this.defaultValue = getter.get();
        }

        double get() { return getter.get(); }

        void set(double v) {
            double clamped = Math.max(min, Math.min(max, v));
            setter.set(Math.round(clamped * 10000.0) / 10000.0);
        }

        void add(double delta) {
            if (toggle) set(get() > 0.5 ? 0 : 1);   // 토글은 증감 대신 뒤집기
            else set(get() + delta);
        }

        void reset() { set(defaultValue); }

        String display() { return toggle ? (get() > 0.5 ? "ON" : "OFF") : fmt(get()); }
    }

    /** 서브시스템 하나 = 파라미터 묶음 */
    private static class Group {
        final String name;
        final List<Param> params = new ArrayList<>();
        int index = 0;

        Group(String name) { this.name = name; }

        Group add(Param p) { params.add(p); return this; }

        Param selected() { return params.get(index); }
    }

    private final List<Group> groups = new ArrayList<>();
    private int groupIndex = 0;

    private static final double[] STEP_SCALES = {0.1, 1.0, 10.0};
    private int stepScaleIndex = 1;

    // 버튼을 누르고 있으면 값이 연속으로 변하도록 하는 리핏 처리
    private static final double HOLD_DELAY_MS = 400;
    private static final double REPEAT_MS = 90;

    private static class Repeat {
        private final ElapsedTime timer = new ElapsedTime();
        private boolean down = false;
        private boolean firstRepeatDone = false;

        boolean fire(boolean pressed) {
            if (!pressed) { down = false; firstRepeatDone = false; return false; }
            if (!down) { down = true; timer.reset(); return true; }
            if (!firstRepeatDone) {
                if (timer.milliseconds() > HOLD_DELAY_MS) { firstRepeatDone = true; timer.reset(); return true; }
                return false;
            }
            if (timer.milliseconds() > REPEAT_MS) { timer.reset(); return true; }
            return false;
        }
    }

    /** 누른 순간에만 한 번 반응 (토글용 - 누르고 있어도 반복되지 않음) */
    private static class Edge {
        private boolean down = false;

        boolean fire(boolean pressed) {
            if (pressed && !down) { down = true; return true; }
            if (!pressed) down = false;
            return false;
        }
    }

    // 튜닝 입력 (이동/증감은 누르고 있으면 반복)
    private final Repeat navUp = new Repeat();
    private final Repeat navDown = new Repeat();
    private final Repeat navLeft = new Repeat();
    private final Repeat navRight = new Repeat();
    private final Repeat valueMinus = new Repeat();
    private final Repeat valuePlus = new Repeat();
    private final Repeat valueMinusFast = new Repeat();
    private final Repeat valuePlusFast = new Repeat();
    private final Edge stepCycle = new Edge();
    private final Edge resetOne = new Edge();
    private final Edge resetAll = new Edge();
    private final Edge helpToggle = new Edge();

    // 실행 입력 (전부 누른 순간 1회)
    private final Edge btnFar = new Edge();
    private final Edge btnClose = new Edge();
    private final Edge btnManual = new Edge();
    private final Edge btnFeed = new Edge();
    private final Edge btnMacro = new Edge();
    private final Edge btnStopAll = new Edge();

    // ==========================================================
    //  OpMode 본체
    // ==========================================================

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        initHardware();
        buildParams();

        // INIT 상태에서도 값 조정이 가능합니다 (모터는 전부 정지 상태).
        while (opModeInInit()) {
            handleTuning();
            telemetry.addLine("=== 튜닝 준비 완료 (START 누르기 전에도 값 조정 가능) ===");
            showParams();
            showHardwareStatus();
            showHelp();
            telemetry.update();
        }

        if (isStopRequested()) return;

        feedTimer.reset();
        macroTimer.reset();
        spinupTimer.reset();

        while (opModeIsActive()) {
            handleTuning();
            handleRunInput();
            applyIntakeAndTransfer();
            applyShooter();
            showTelemetry();
            telemetry.update();
        }

        stopAll();
    }

    // ----------------------------------------------------------
    //  초기화
    // ----------------------------------------------------------

    private void initHardware() {
        intakeMotor = getMotor("eater1");
        transferMotor = getMotor("eater2");
        shooter1 = getMotor("shooter1");
        shooter2 = getMotor("shooter2");

        sweeperLeft = getServo("sweeperLeft");
        sweeperRight = getServo("sweeperRight");

        try {
            ballSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
        } catch (Exception e) {
            ballSensor = null;
            missingDevices.add("colorSensor");
        }

        // 인테이크 / 트랜스퍼: 파워 제어
        for (DcMotorEx m : new DcMotorEx[]{intakeMotor, transferMotor}) {
            if (m == null) continue;
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // 슈터: 속도 제어
        for (DcMotorEx m : new DcMotorEx[]{shooter1, shooter2}) {
            if (m == null) continue;
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        // 기본 PIDF 저장 (usePidf를 껐을 때 되돌리기 위함)
        if (shooter1 != null) {
            try {
                stockPidf = shooter1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            } catch (Exception e) {
                stockPidf = null;
            }
        }
    }

    private DcMotorEx getMotor(String name) {
        try {
            return hardwareMap.get(DcMotorEx.class, name);
        } catch (Exception e) {
            missingDevices.add(name);
            return null;
        }
    }

    private CRServo getServo(String name) {
        try {
            return hardwareMap.get(CRServo.class, name);
        } catch (Exception e) {
            missingDevices.add(name);
            return null;
        }
    }

    /** 조정 가능한 값 목록을 서브시스템별로 구성합니다. */
    private void buildParams() {
        Group intake = new Group("인테이크 (INTAKE)");
        intake.add(new Param("인테이크 파워", "part/Constants.EATER_POWER", 0.05, -1, 1, false,
                () -> INTAKE.power, v -> INTAKE.power = v));
        intake.add(new Param("인테이크 역방향 파워", "part/Constants.EATER_REVERSE_POWER", 0.05, -1, 1, false,
                () -> INTAKE.reversePower, v -> INTAKE.reversePower = v));
        intake.add(new Param("스위퍼 파워", "Eater.update() sweeper", 0.05, -1, 1, false,
                () -> INTAKE.sweeperPower, v -> INTAKE.sweeperPower = v));
        intake.add(new Param("스위퍼 역방향 파워", "Eater.update() sweeper", 0.05, -1, 1, false,
                () -> INTAKE.sweeperReversePower, v -> INTAKE.sweeperReversePower = v));
        intake.add(new Param("스위퍼 인테이크 동조", "Eater.update()", 1, 0, 1, true,
                () -> INTAKE.sweeperFollowIntake ? 1 : 0, v -> INTAKE.sweeperFollowIntake = v > 0.5));
        intake.add(new Param("공 감지 거리(cm)", "part/Constants.EATER_BALL_DISTANCE_CM", 0.1, 0, 30, false,
                () -> INTAKE.ballDistanceCm, v -> INTAKE.ballDistanceCm = v));
        intake.add(new Param("감지 해제 여유(cm)", "Eater.update() 의 +2.0", 0.1, 0, 20, false,
                () -> INTAKE.ballClearMarginCm, v -> INTAKE.ballClearMarginCm = v));
        intake.add(new Param("eater1 방향 반전", "Eater.init()", 1, 0, 1, true,
                () -> INTAKE.motorReverse ? 1 : 0, v -> INTAKE.motorReverse = v > 0.5));
        intake.add(new Param("sweeperLeft 반전", "Eater.init()", 1, 0, 1, true,
                () -> INTAKE.sweeperLeftReverse ? 1 : 0, v -> INTAKE.sweeperLeftReverse = v > 0.5));
        intake.add(new Param("sweeperRight 반전", "Eater.init()", 1, 0, 1, true,
                () -> INTAKE.sweeperRightReverse ? 1 : 0, v -> INTAKE.sweeperRightReverse = v > 0.5));
        groups.add(intake);

        Group transfer = new Group("트랜스퍼 (TRANSFER)");
        transfer.add(new Param("트랜스퍼 파워", "part/Constants.EATER_POWER", 0.05, -1, 1, false,
                () -> TRANSFER.power, v -> TRANSFER.power = v));
        transfer.add(new Param("트랜스퍼 역방향 파워", "part/Constants.EATER_REVERSE_POWER", 0.05, -1, 1, false,
                () -> TRANSFER.reversePower, v -> TRANSFER.reversePower = v));
        transfer.add(new Param("급탄 파워", "Eater.update() FEEDING", 0.05, -1, 1, false,
                () -> TRANSFER.feedPower, v -> TRANSFER.feedPower = v));
        transfer.add(new Param("급탄 시간(ms)", "part/Constants.EATER_FEED_TIME_MS", 10, 0, 3000, false,
                () -> TRANSFER.feedTimeMs, v -> TRANSFER.feedTimeMs = v));
        transfer.add(new Param("급탄 중 인테이크 동작", "Eater.update() FEEDING", 1, 0, 1, true,
                () -> TRANSFER.feedRunsIntake ? 1 : 0, v -> TRANSFER.feedRunsIntake = v > 0.5));
        transfer.add(new Param("eater2 방향 반전", "Eater.init()", 1, 0, 1, true,
                () -> TRANSFER.motorReverse ? 1 : 0, v -> TRANSFER.motorReverse = v > 0.5));
        groups.add(transfer);

        Group shoot = new Group("슈터 (SHOOTER)");
        shoot.add(new Param("파슛 속도", "part/Constants.FAR_SHOOT_VELOCITY", 25, 0, 3200, false,
                () -> SHOOTER.farVelocity, v -> SHOOTER.farVelocity = v));
        shoot.add(new Param("골밑슛 속도", "part/Constants.CLOSE_SHOOT_VELOCITY", 25, 0, 3200, false,
                () -> SHOOTER.closeVelocity, v -> SHOOTER.closeVelocity = v));
        shoot.add(new Param("대기 유지 속도", "(테스트 전용)", 25, 0, 3200, false,
                () -> SHOOTER.idleVelocity, v -> SHOOTER.idleVelocity = v));
        shoot.add(new Param("수동 파워", "Shooter.manualShooter()", 0.05, -1, 1, false,
                () -> SHOOTER.manualPower, v -> SHOOTER.manualPower = v));
        shoot.add(new Param("속도 허용 오차", "part/Constants.VEL_TOLERANCE", 5, 0, 500, false,
                () -> SHOOTER.velTolerance, v -> SHOOTER.velTolerance = v));
        shoot.add(new Param("예열 시간(ms)", "part/Constants.SHOOTER_SPOOL_TIME_MS", 50, 0, 10000, false,
                () -> SHOOTER.spoolTimeMs, v -> SHOOTER.spoolTimeMs = v));
        shoot.add(new Param("속도 도달 시 즉시 발사", "(테스트 전용)", 1, 0, 1, true,
                () -> SHOOTER.spoolWaitForAtSpeed ? 1 : 0, v -> SHOOTER.spoolWaitForAtSpeed = v > 0.5));
        shoot.add(new Param("파슛 연사 간격(ms)", "part/Constants.RAPID_FIRE_DELAY_FAR_MS", 10, 0, 3000, false,
                () -> SHOOTER.rapidDelayFarMs, v -> SHOOTER.rapidDelayFarMs = v));
        shoot.add(new Param("골밑슛 연사 간격(ms)", "part/Constants.RAPID_FIRE_DELAY_CLOSE_MS", 10, 0, 3000, false,
                () -> SHOOTER.rapidDelayCloseMs, v -> SHOOTER.rapidDelayCloseMs = v));
        shoot.add(new Param("커스텀 PIDF 사용", "(테스트 전용)", 1, 0, 1, true,
                () -> SHOOTER.usePidf ? 1 : 0, v -> SHOOTER.usePidf = v > 0.5));
        shoot.add(new Param("kP", "(테스트 전용)", 0.1, 0, 100, false,
                () -> SHOOTER.kP, v -> SHOOTER.kP = v));
        shoot.add(new Param("kI", "(테스트 전용)", 0.01, 0, 50, false,
                () -> SHOOTER.kI, v -> SHOOTER.kI = v));
        shoot.add(new Param("kD", "(테스트 전용)", 0.1, 0, 100, false,
                () -> SHOOTER.kD, v -> SHOOTER.kD = v));
        shoot.add(new Param("kF", "(테스트 전용)", 0.1, 0, 100, false,
                () -> SHOOTER.kF, v -> SHOOTER.kF = v));
        shoot.add(new Param("shooter1 방향 반전", "Shooter.init()", 1, 0, 1, true,
                () -> SHOOTER.motor1Reverse ? 1 : 0, v -> SHOOTER.motor1Reverse = v > 0.5));
        shoot.add(new Param("shooter2 방향 반전", "Shooter.init()", 1, 0, 1, true,
                () -> SHOOTER.motor2Reverse ? 1 : 0, v -> SHOOTER.motor2Reverse = v > 0.5));
        groups.add(shoot);
    }

    // ----------------------------------------------------------
    //  튜닝 입력 처리 (조종기 1, 2 양쪽 모두 사용 가능)
    // ----------------------------------------------------------

    private void handleTuning() {
        if (navLeft.fire(gamepad1.dpad_left || gamepad2.dpad_left)) {
            groupIndex = (groupIndex + groups.size() - 1) % groups.size();
        }
        if (navRight.fire(gamepad1.dpad_right || gamepad2.dpad_right)) {
            groupIndex = (groupIndex + 1) % groups.size();
        }

        Group group = groups.get(groupIndex);

        if (navUp.fire(gamepad1.dpad_up || gamepad2.dpad_up)) {
            group.index = (group.index + group.params.size() - 1) % group.params.size();
        }
        if (navDown.fire(gamepad1.dpad_down || gamepad2.dpad_down)) {
            group.index = (group.index + 1) % group.params.size();
        }

        if (stepCycle.fire(gamepad2.x)) {
            stepScaleIndex = (stepScaleIndex + 1) % STEP_SCALES.length;
        }

        Param param = group.selected();
        double delta = param.step * STEP_SCALES[stepScaleIndex];

        if (valueMinus.fire(gamepad1.left_stick_button || gamepad2.left_bumper)) param.add(-delta);
        if (valuePlus.fire(gamepad1.right_stick_button || gamepad2.right_bumper)) param.add(delta);
        if (valueMinusFast.fire(gamepad2.left_trigger > 0.5)) param.add(-delta * 10);
        if (valuePlusFast.fire(gamepad2.right_trigger > 0.5)) param.add(delta * 10);

        if (resetOne.fire(gamepad2.y)) param.reset();
        if (resetAll.fire(gamepad2.back)) {
            for (Group g : groups) {
                for (Param p : g.params) p.reset();
            }
        }
        if (helpToggle.fire(gamepad2.start)) showHelp = !showHelp;
    }

    // ----------------------------------------------------------
    //  실행 입력 처리 (조종기 1 전용)
    // ----------------------------------------------------------

    private void handleRunInput() {
        if (btnFar.fire(gamepad1.y)) {
            shootMode = (shootMode == ShootMode.FAR) ? ShootMode.STOP : ShootMode.FAR;
        }
        if (btnClose.fire(gamepad1.a)) {
            shootMode = (shootMode == ShootMode.CLOSE) ? ShootMode.STOP : ShootMode.CLOSE;
        }
        if (btnManual.fire(gamepad1.x)) {
            shootMode = (shootMode == ShootMode.MANUAL) ? ShootMode.STOP : ShootMode.MANUAL;
        }

        // 1회 급탄
        if (btnFeed.fire(gamepad1.b) && !feeding) startFeed();

        // 발사 시퀀스(예열 -> 급탄 -> 연사) 토글
        if (btnMacro.fire(gamepad1.start)) {
            if (macro == Macro.OFF) {
                if (shootMode == ShootMode.STOP || shootMode == ShootMode.MANUAL) shootMode = ShootMode.FAR;
                macro = Macro.SPOOL;
                macroTimer.reset();
            } else {
                macro = Macro.OFF;
            }
        }

        // 전체 정지
        if (btnStopAll.fire(gamepad1.back)) {
            macro = Macro.OFF;
            feeding = false;
            shootMode = ShootMode.STOP;
        }

        // 급탄 종료 처리
        if (feeding && feedTimer.milliseconds() >= TRANSFER.feedTimeMs) {
            feeding = false;
            // 급탄이 끝난 시점부터 속도 회복 시간 측정 시작
            recovering = true;
            recoveryTimer.reset();
            lastRecoveryMs = -1;
        }
    }

    private void startFeed() {
        feeding = true;
        feedTimer.reset();
        minVelAfterFeed = -1;
    }

    /** 발사 시퀀스: 예열 -> 급탄 -> 연사 간격 대기 -> 급탄 ... */
    private void updateMacro() {
        double rapidDelay = (shootMode == ShootMode.CLOSE)
                ? SHOOTER.rapidDelayCloseMs : SHOOTER.rapidDelayFarMs;

        switch (macro) {
            case SPOOL:
                boolean spoolDone = SHOOTER.spoolWaitForAtSpeed
                        ? (atSpeed || macroTimer.milliseconds() >= SHOOTER.spoolTimeMs)
                        : (macroTimer.milliseconds() >= SHOOTER.spoolTimeMs);
                if (spoolDone) {
                    startFeed();
                    macroTimer.reset();
                    macro = Macro.FEED;
                }
                break;

            case FEED:
                if (!feeding) {
                    macroTimer.reset();
                    macro = Macro.WAIT;
                }
                break;

            case WAIT:
                if (macroTimer.milliseconds() >= rapidDelay) {
                    startFeed();
                    macroTimer.reset();
                    macro = Macro.FEED;
                }
                break;

            case OFF:
            default:
                break;
        }
    }

    // ----------------------------------------------------------
    //  하드웨어 출력
    // ----------------------------------------------------------

    private void applyIntakeAndTransfer() {
        double distanceCm = (ballSensor != null) ? ballSensor.getDistance(DistanceUnit.CM) : -1;

        // 공 감지 (Eater.update()와 동일한 히스테리시스 로직)
        if (distanceCm >= 0) {
            if (distanceCm > INTAKE.ballDistanceCm + INTAKE.ballClearMarginCm) ballDetected = false;
            else if (distanceCm < INTAKE.ballDistanceCm) ballDetected = true;
        }

        intakePower = 0;
        transferPower = 0;

        if (gamepad1.left_trigger > 0.1) intakePower = INTAKE.power;
        else if (gamepad1.left_bumper) intakePower = INTAKE.reversePower;

        if (gamepad1.right_trigger > 0.1) transferPower = TRANSFER.power;
        else if (gamepad1.right_bumper) transferPower = TRANSFER.reversePower;

        // 급탄 중에는 트랜스퍼(및 설정 시 인테이크)를 강제로 돌립니다.
        if (feeding) {
            transferPower = TRANSFER.feedPower;
            if (TRANSFER.feedRunsIntake) intakePower = INTAKE.power;
        }

        // 스위퍼: 왼쪽 스틱으로 단독 조작, 스틱이 중립이면 인테이크와 동조
        double stick = -gamepad1.left_stick_y;
        if (Math.abs(stick) > 0.15) {
            sweeperPower = stick;
        } else if (INTAKE.sweeperFollowIntake) {
            if (intakePower > 0) sweeperPower = INTAKE.sweeperPower;
            else if (intakePower < 0) sweeperPower = INTAKE.sweeperReversePower;
            else sweeperPower = 0;
        } else {
            sweeperPower = 0;
        }

        if (intakeMotor != null) {
            intakeMotor.setDirection(INTAKE.motorReverse
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            intakeMotor.setPower(intakePower);
        }
        if (transferMotor != null) {
            transferMotor.setDirection(TRANSFER.motorReverse
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            transferMotor.setPower(transferPower);
        }
        if (sweeperLeft != null) {
            sweeperLeft.setDirection(INTAKE.sweeperLeftReverse
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            sweeperLeft.setPower(sweeperPower);
        }
        if (sweeperRight != null) {
            sweeperRight.setDirection(INTAKE.sweeperRightReverse
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            sweeperRight.setPower(sweeperPower);
        }
    }

    private void applyShooter() {
        if (shooter1 != null) {
            shooter1.setDirection(SHOOTER.motor1Reverse
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        }
        if (shooter2 != null) {
            shooter2.setDirection(SHOOTER.motor2Reverse
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        }

        applyPidfIfChanged();

        switch (shootMode) {
            case FAR:    targetVelocity = SHOOTER.farVelocity; break;
            case CLOSE:  targetVelocity = SHOOTER.closeVelocity; break;
            case MANUAL: targetVelocity = 0; break;
            case STOP:
            default:     targetVelocity = SHOOTER.idleVelocity; break;
        }

        double actual = (shooter1 != null) ? shooter1.getVelocity() : 0;
        atSpeed = targetVelocity > 0 && Math.abs(actual - targetVelocity) <= SHOOTER.velTolerance;

        // 목표가 바뀌면 예열 실측 타이머를 다시 시작합니다.
        if (Math.abs(targetVelocity - lastTarget) > 1e-6) {
            lastTarget = targetVelocity;
            spinupTimer.reset();
            spinupMeasured = (targetVelocity <= 0);
            lastSpinupMs = -1;
        }
        if (!spinupMeasured && atSpeed) {
            lastSpinupMs = spinupTimer.milliseconds();
            spinupMeasured = true;
        }

        // 급탄 중 속도 낙폭 / 급탄 후 회복 시간 측정
        if (feeding && targetVelocity > 0) {
            if (minVelAfterFeed < 0 || actual < minVelAfterFeed) minVelAfterFeed = actual;
        }
        if (recovering) {
            if (targetVelocity <= 0) recovering = false;
            else if (atSpeed) {
                lastRecoveryMs = recoveryTimer.milliseconds();
                recovering = false;
            }
        }

        updateMacro();

        if (shootMode == ShootMode.MANUAL) {
            if (shooter1 != null) shooter1.setPower(SHOOTER.manualPower);
            if (shooter2 != null) shooter2.setPower(SHOOTER.manualPower);
        } else {
            if (shooter1 != null) shooter1.setVelocity(targetVelocity);
            if (shooter2 != null) shooter2.setVelocity(targetVelocity);
        }
    }

    /** PIDF는 값이 바뀔 때만 허브로 전송합니다 (매 루프 전송하면 통신 부하가 큽니다). */
    private void applyPidfIfChanged() {
        double p, i, d, f;
        if (SHOOTER.usePidf) {
            p = SHOOTER.kP; i = SHOOTER.kI; d = SHOOTER.kD; f = SHOOTER.kF;
        } else if (stockPidf != null) {
            p = stockPidf.p; i = stockPidf.i; d = stockPidf.d; f = stockPidf.f;
        } else {
            return; // 기본값을 못 읽었으면 건드리지 않습니다.
        }

        if (p == appliedP && i == appliedI && d == appliedD && f == appliedF) return;

        try {
            if (shooter1 != null) shooter1.setVelocityPIDFCoefficients(p, i, d, f);
            if (shooter2 != null) shooter2.setVelocityPIDFCoefficients(p, i, d, f);
            pidfError = null;
        } catch (Exception e) {
            pidfError = e.getMessage();
        }
        // 실패해도 갱신해서 매 루프 재시도하지 않도록 합니다.
        appliedP = p; appliedI = i; appliedD = d; appliedF = f;
    }

    private void stopAll() {
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (transferMotor != null) transferMotor.setPower(0);
        if (sweeperLeft != null) sweeperLeft.setPower(0);
        if (sweeperRight != null) sweeperRight.setPower(0);
        if (shooter1 != null) shooter1.setVelocity(0);
        if (shooter2 != null) shooter2.setVelocity(0);
    }

    // ----------------------------------------------------------
    //  텔레메트리
    // ----------------------------------------------------------

    private void showTelemetry() {
        showParams();

        telemetry.addLine();
        telemetry.addLine(String.format(Locale.US, "[슈터] 모드 %s | 시퀀스 %s", shootMode, macro));
        telemetry.addData("  목표 속도", fmt(targetVelocity));
        telemetry.addData("  실제 속도", "%s / %s",
                shooter1 != null ? fmt(shooter1.getVelocity()) : "-",
                shooter2 != null ? fmt(shooter2.getVelocity()) : "-");
        telemetry.addData("  오차", shooter1 != null ? fmt(shooter1.getVelocity() - targetVelocity) : "-");
        telemetry.addData("  목표 도달", atSpeed ? "O" : "X");
        telemetry.addData("  예열 실측(ms)", (lastSpinupMs < 0 ? "측정중" : fmt(lastSpinupMs))
                + "   <- 예열 시간 참고값");
        telemetry.addData("  급탄 후 회복(ms)", (lastRecoveryMs < 0 ? "-" : fmt(lastRecoveryMs))
                + "   <- 연사 간격 참고값");
        telemetry.addData("  급탄 중 최저 속도", minVelAfterFeed < 0 ? "-" : fmt(minVelAfterFeed));
        telemetry.addData("  PIDF", SHOOTER.usePidf
                ? String.format(Locale.US, "커스텀 %s / %s / %s / %s",
                        fmt(appliedP), fmt(appliedI), fmt(appliedD), fmt(appliedF))
                : (stockPidf != null
                        ? String.format(Locale.US, "기본 %.2f / %.2f / %.2f / %.2f",
                                stockPidf.p, stockPidf.i, stockPidf.d, stockPidf.f)
                        : "읽기 실패"));
        if (pidfError != null) telemetry.addData("  PIDF 오류", pidfError);

        telemetry.addLine();
        telemetry.addLine("[이터]");
        telemetry.addData("  인테이크 파워", fmt(intakePower));
        telemetry.addData("  트랜스퍼 파워", fmt(transferPower) + (feeding ? "   (급탄중)" : ""));
        telemetry.addData("  스위퍼 파워", fmt(sweeperPower));
        telemetry.addData("  거리(cm)", ballSensor != null
                ? fmt(ballSensor.getDistance(DistanceUnit.CM)) : "센서 없음");
        telemetry.addData("  공 감지", ballDetected ? "O" : "X");

        telemetry.addLine();
        telemetry.addData("[전류A] eater1 / eater2", "%s / %s", current(intakeMotor), current(transferMotor));
        telemetry.addData("[전류A] shooter1 / shooter2", "%s / %s", current(shooter1), current(shooter2));

        showHardwareStatus();
        showHelp();
    }

    private void showParams() {
        Group group = groups.get(groupIndex);
        telemetry.addLine(String.format(Locale.US, "===== 튜닝 [%d/%d] %s   (스텝 x%s) =====",
                groupIndex + 1, groups.size(), group.name, fmt(STEP_SCALES[stepScaleIndex])));

        for (int i = 0; i < group.params.size(); i++) {
            Param p = group.params.get(i);
            telemetry.addLine(String.format(Locale.US, "%s %-22s %s",
                    i == group.index ? ">" : " ", p.name, p.display()));
        }
        telemetry.addLine("  -> 적용 위치: " + group.selected().target);
    }

    private void showHardwareStatus() {
        if (!missingDevices.isEmpty()) {
            telemetry.addLine();
            telemetry.addLine("!! 설정에 없는 장치: " + missingDevices + " (해당 기능 비활성화)");
        }
    }

    private void showHelp() {
        if (!showHelp) {
            telemetry.addLine();
            telemetry.addLine("[조작법 숨김 - 조종기2 START로 다시 표시]");
            return;
        }
        telemetry.addLine();
        telemetry.addLine("===== 조작법 (조종기2 START: 숨기기) =====");
        telemetry.addLine("[실행 - 조종기1]");
        telemetry.addLine("  LT / LB : 인테이크 정방향 / 역방향");
        telemetry.addLine("  RT / RB : 트랜스퍼 정방향 / 역방향");
        telemetry.addLine("  왼쪽 스틱 상하 : 스위퍼 단독 조작");
        telemetry.addLine("  Y : 파슛 속도 | A : 골밑슛 속도 | X : 수동 파워 (각각 토글)");
        telemetry.addLine("  B : 1회 급탄 | START : 발사 시퀀스 토글 | BACK : 전체 정지");
        telemetry.addLine("[튜닝 - 조종기1 / 2 공통]");
        telemetry.addLine("  십자키 좌우 : 그룹(인테이크/트랜스퍼/슈터) 선택");
        telemetry.addLine("  십자키 상하 : 항목 선택");
        telemetry.addLine("  조종기1 L3 / R3, 조종기2 LB / RB : 값 - / +");
        telemetry.addLine("[튜닝 - 조종기2 전용]");
        telemetry.addLine("  LT / RT : 값 - / + (10배) | X : 스텝 배율");
        telemetry.addLine("  Y : 현재 항목 초기화 | BACK : 전체 초기화");
    }

    private String current(DcMotorEx motor) {
        if (motor == null) return "-";
        try {
            return String.format(Locale.US, "%.1f", motor.getCurrent(CurrentUnit.AMPS));
        } catch (Exception e) {
            return "-";
        }
    }

    /** 소수점 뒤 불필요한 0을 지운 짧은 숫자 표기 */
    private static String fmt(double v) {
        String s = String.format(Locale.US, "%.4f", v);
        if (s.contains(".")) {
            while (s.endsWith("0")) s = s.substring(0, s.length() - 1);
            if (s.endsWith(".")) s = s.substring(0, s.length() - 1);
        }
        return s;
    }
}
