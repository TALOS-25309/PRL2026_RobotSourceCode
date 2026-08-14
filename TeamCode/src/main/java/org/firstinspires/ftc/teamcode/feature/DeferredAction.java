package org.firstinspires.ftc.teamcode.feature;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.function.Supplier;

public class DeferredAction implements Action {
    private final Supplier<Action> factory;
    private Action built = null;

    public DeferredAction(Supplier<Action> factory) { this.factory = factory; }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (built == null) built = factory.get();
        return built.run(packet);
    }
}