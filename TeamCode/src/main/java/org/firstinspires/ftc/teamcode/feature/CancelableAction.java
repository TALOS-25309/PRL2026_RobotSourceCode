package org.firstinspires.ftc.teamcode.feature;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;


public class CancelableAction implements Action {
    private final MecanumDrive drive;
    private final Action inner;
    private final BooleanSupplier cancelWhen;

    public CancelableAction(MecanumDrive drive, BooleanSupplier cancelWhen, Action inner) {
        this.drive = drive; this.cancelWhen = cancelWhen; this.inner = inner;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (cancelWhen.getAsBoolean()) {
            // 중단 시 모터 파워가 마지막 값으로 남으므로 반드시 0으로 눌러줘야 함
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            return false;   // false = 이 Action 종료 → SequentialAction의 다음 단계로
        }
        return inner.run(packet);
    }
}

