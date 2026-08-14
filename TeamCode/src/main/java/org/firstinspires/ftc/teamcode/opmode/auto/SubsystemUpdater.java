package org.firstinspires.ftc.teamcode.opmode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.part.Part;

/**
 * Executes part updates in the background during RoadRunner sequential actions.
 */
public class SubsystemUpdater implements Action {
    private final Part[] parts;
    public boolean keepRunning = true;

    public SubsystemUpdater(Part... parts) {
        this.parts = parts;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        for (Part part : parts) {
            part.update();
        }
        
        return keepRunning; 
    }
}
