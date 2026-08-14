package org.firstinspires.ftc.teamcode.part;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Vision implements Part {
    
    private Limelight3A limelight;
    private Telemetry telemetry;
    
    // 타겟 상태 및 위치 데이터
    private boolean hasTarget = false;
    private double tx = 0.0; // 가로 오차 각도 (도)
    private double ty = 0.0; // 세로 오차 각도 (도)
    private double ta = 0.0; // 타겟 면적 (%)
    
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        
        // Limelight 하드웨어 매핑
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        
        // 라임라이트 설정
        limelight.setPollRateHz(100); // 100Hz 갱신
        
        // 카메라를 켜고 파이프라인 처리를 시작합니다.
        limelight.start();
    }
    
    @Override
    public void start() {
        // 이미 init()에서 start()를 호출했으므로 여기서는 비워둡니다.
    }
    
    @Override
    public void update() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("LL Name", "%s", status.getName());
        telemetry.addData("LL Temp", "%.1fC", status.getTemp());
        telemetry.addData("LL FPS", "%.1f", status.getFps());
        
        // 가장 최근의 비전 분석 결과를 가져옵니다.
        LLResult result = limelight.getLatestResult();
        
        if (result != null && result.isValid()) {
            hasTarget = true;
            tx = result.getTx();
            ty = result.getTy();
            ta = result.getTa();
            
            telemetry.addData("Vision Target", "🎯 감지됨!");
            telemetry.addData("tx (가로 오차)", "%.2f 도", tx);
            telemetry.addData("ty (세로 오차)", "%.2f 도", ty);
        } else {
            hasTarget = false;
            tx = 0.0;
            ty = 0.0;
            ta = 0.0;
            telemetry.addData("Vision Target", "None");
        }
    }
    
    @Override
    public void stop() {
        limelight.stop();
    }
    
    // --- 외부 제어용 Getter 메서드 ---
    public boolean hasTarget() {
        return hasTarget;
    }
    
    public double getTx() {
        return tx;
    }
    
    public double getTy() {
        return ty;
    }

    // 파이프라인 스위칭 (0: Goal, 1: Ball)
    public void setPipeline(int id) {
        limelight.pipelineSwitch(id);
    }
}
