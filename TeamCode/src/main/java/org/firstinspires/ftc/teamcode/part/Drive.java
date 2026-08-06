package org.firstinspires.ftc.teamcode.part;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class Drive implements Part {

    public MecanumDrive mecanumDrive;
    private Telemetry telemetry;

    // 현재 로봇이 받아야 할 주행 파워 (x: 직진, y: 좌우, heading: 회전)
    private PoseVelocity2d drivePower = new PoseVelocity2d(new Vector2d(0, 0), 0);

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        
        // RoadRunner의 MecanumDrive 인스턴스를 생성합니다. (초기 위치는 0,0,0으로 설정)
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }


    @Override
    public void start() {
        setDrivePower(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    @Override
    public void update() {
        // 1. 외부에서 설정된 계산된 파워를 모터에 인가합니다.
        mecanumDrive.setDrivePowers(drivePower);

        // 2. 로봇의 현재 위치(Odometry)를 지속적으로 추적/업데이트합니다.
        // 이 코드가 있어야 나중에 필드 중심 주행이나 자동화 매크로가 정상 작동합니다.
        mecanumDrive.updatePoseEstimate();

        // 3. 텔레메트리에 현재 위치 좌표를 출력합니다.
        telemetry.addData("Drive X", mecanumDrive.localizer.getPose().position.x);
        telemetry.addData("Drive Y", mecanumDrive.localizer.getPose().position.y);
        telemetry.addData("Drive Heading", Math.toDegrees(mecanumDrive.localizer.getPose().heading.toDouble()));
    }

    @Override
    public void stop() {
        setDrivePower(new PoseVelocity2d(new Vector2d(0, 0), 0));
        mecanumDrive.setDrivePowers(drivePower);
    }

    /**
     * 외부(OpMode)에서 조이스틱 값을 넘겨주어 주행 파워를 설정합니다.
     * @param power (x: 전후, y: 좌우, heading: 회전)
     */
    public void setDrivePower(PoseVelocity2d power) {
        this.drivePower = power;
    }
}
