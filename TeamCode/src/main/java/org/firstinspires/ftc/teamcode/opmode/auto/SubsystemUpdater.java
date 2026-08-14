package org.firstinspires.ftc.teamcode.opmode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.part.Part;

/**
 * RoadRunner 1.0의 자율주행(Actions.runBlocking) 루프 속에서 
 * 텔레옵용 서브시스템들(Eater, Shooter 등)의 update()를 
 * 백그라운드로 계속 호출해주는 병렬 처리용 Action입니다.
 */
public class SubsystemUpdater implements Action {
    private final Part[] parts;
    public boolean keepRunning = true; // 이 값이 false가 되면 백그라운드 업데이트가 종료됩니다.

    public SubsystemUpdater(Part... parts) {
        this.parts = parts;
        this.keepRunning = true;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        // 등록된 모든 부품들의 상태 머신(update)을 한 번씩 실행합니다.
        for (Part part : parts) {
            part.update();
        }
        
        // true를 반환하면 RoadRunner가 이 Action을 끝내지 않고 다음 루프 때 또 호출합니다.
        // keepRunning이 false가 되면 이 Action은 종료되며, RoadRunner의 runBlocking 루프도 정상적으로 끝나게 됩니다.
        return keepRunning; 
    }
}
