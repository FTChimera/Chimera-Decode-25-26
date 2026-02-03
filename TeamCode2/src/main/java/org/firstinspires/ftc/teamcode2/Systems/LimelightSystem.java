package org.firstinspires.ftc.teamcode2.Systems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@SuppressWarnings("SpellCheckingInspection")
public class LimelightSystem {
    public LimelightSystem(HardwareMap hwMap) {
        limelight1 = hwMap.get(Limelight3A.class, "limelight");
    }
    private static int changePipeline(int curPipe, boolean isRed) {
        // CHANGE PIPELINES
        switch (curPipe) {
            case 1:
                return 2; // purple → green
            case 2:
                return 3; // green → obelisk
            case 3:
                return isRed ? 4 : 5; // obelisk → red or blue
            case 4:
                return 1;
            case 5:
                return 1; // back to purple
            default:
                return 1; // start at purple
        }
    }

    private static double calculateDistanceCurve(double x) {
        return 0;
    }

    private static boolean isDisconnected(LLResult newR) {
        return !newR.isValid() || newR==null;
    }

    public double tx=0,ty=0,ta=0,tid=0, dist=0;
    public boolean isDisconnected;
    public Limelight3A limelight1;
    public Pose3D botpose;
    public LLResult result;
    public void start(int pipeline) {startLLWithPipeline(pipeline);}
    private void startLLWithPipeline(int pipeline){limelight1.start();limelight1.pipelineSwitch(pipeline);}
    public int pipelineChange(Constants.AllianceColor allianceColor){int newPipe = changePipeline(limelight1.getLatestResult().getPipelineIndex(), allianceColor== Constants.AllianceColor.RED);limelight1.pipelineSwitch(newPipe);return newPipe;}
    public void LLUpdate() {
        result = limelight1.getLatestResult();
        if (result!=null && result.isValid()) {
            tx=result.getTx();
            ty= result.getTy();
            ta=result.getTa();
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiduciary : fiducials ) {tid = fiduciary.getFiducialId();}
            dist = calculateDistanceCurve(ta);
            botpose = result.getBotpose_MT2();
            isDisconnected = isDisconnected(result);
        }
    }
    public double getLLScore() {
        return Math.abs(tx);
    }

}

