package org.firstinspires.ftc.teamcode.part;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Vision implements Part {
    
    private Limelight3A limelight;
    private Telemetry telemetry;
    
    private boolean hasTarget = false;
    private double tx = 0.0;
    private double ty = 0.0;
    private double ta = 0.0;
    
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }
    
    @Override
    public void start() {}
    
    @Override
    public void update() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("LL Name", "%s", status.getName());
        telemetry.addData("LL Temp", "%.1fC", status.getTemp());
        telemetry.addData("LL FPS", "%.1f", status.getFps());
        
        LLResult result = limelight.getLatestResult();
        
        if (result != null && result.isValid()) {
            hasTarget = true;
            tx = result.getTx();
            ty = result.getTy();
            ta = result.getTa();
            
            telemetry.addData("Vision Target", "Detected");
            telemetry.addData("tx", "%.2f", tx);
            telemetry.addData("ty", "%.2f", ty);
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
    
    public boolean hasTarget() {
        return hasTarget;
    }
    
    public double getTx() {
        return tx;
    }
    
    public double getTy() {
        return ty;
    }
    
    public void setPipeline(int id) {
        limelight.pipelineSwitch(id);
    }
}
