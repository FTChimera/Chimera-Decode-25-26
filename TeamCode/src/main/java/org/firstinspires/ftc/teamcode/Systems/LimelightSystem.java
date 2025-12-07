package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class LimelightSystem {

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
        return 24.81797
                + (-34825000 - 24.81797)
                / (1 + Math.pow(x / 2.298814e-10, 0.6433497));
    }

    private static boolean isDisconnected(LLResult newR, LLResult oldR) {
        return newR == oldR;
    }
    public static class ChimeraLL {
        public double tx=0,ty=0,ta=0,tid=0, dist=0;
        public boolean isDisconnected;
        public Limelight3A limelight1;
        public Pose3D botpose;
        public LLResult result, old_Result;
        public void setDevice(Limelight3A device) {limelight1=device;}
        public void startLLWithPipeline(int pipeline){limelight1.start();limelight1.pipelineSwitch(pipeline);}
        public int pipelineChange(Consts.AllianceColor allianceColor){int newPipe = changePipeline(limelight1.getLatestResult().getPipelineIndex(), allianceColor==Consts.AllianceColor.RED);limelight1.pipelineSwitch(newPipe);return newPipe;}
        public void LLUpdate() {
            old_Result = result;
            result = limelight1.getLatestResult();
            if (result!=null && result.isValid()) {
                tx=result.getTx();
                ty= result.getTy();
                ta=result.getTa();
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiduciary : fiducials ) {tid = fiduciary.getFiducialId();}
                dist = calculateDistanceCurve(ta);
                botpose = result.getBotpose_MT2();
                isDisconnected = isDisconnected(result, old_Result);
            }
        }
        public double getLLScore() {
            return Math.abs(tx) + Math.abs(dist);
        }
    }
}

