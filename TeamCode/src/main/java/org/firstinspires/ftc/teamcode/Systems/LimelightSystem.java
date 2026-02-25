package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@SuppressWarnings("SpellCheckingInspection")
public class LimelightSystem {
    private final boolean isBeingUsed;
    public LimelightSystem(HardwareMap hwMap) {
        limelight1 = hwMap.get(Limelight3A.class, "limelight");
        isBeingUsed = true;
    }
    public LimelightSystem() {
        isBeingUsed = false;
    }
//    private static int changePipeline(int curPipe, boolean isRed) {
//        // CHANGE PIPELINES
//        switch (curPipe) {
//            case 1:
//                return 2; // purple → green
//            case 2:
//                return 3; // green → obelisk
//            case 3:
//                return isRed ? 4 : 5; // obelisk → red or blue
//            case 4:
//                return 1;
//            case 5:
//                return 1; // back to purple
//            default:
//                return 1; // start at purple
//        }
//    }

    private static double calculateDistanceFromArea(double x) {
        return 1/x;
    }
    public static double calculateDistanceFromRelativePose(LLResultTypes.FiducialResult fiducial) {
        Pose3D pose = fiducial.getRobotPoseTargetSpace();
        double x = pose.getPosition().x;
        double y = pose.getPosition().y;
        return Math.sqrt(x*x + y*y);
    }
    public double tx=0,ty=0,ta=0,tid=0, dist=0;
    public boolean isDisconnected;
    public Limelight3A limelight1;
    private Timer disconnectedTimer;
    public Pose3D botpose;
    public LLResult result;
    private List<LLResultTypes.FiducialResult> fiducials;
    public void start(int pipeline) {startLLWithPipeline(pipeline);}
    private void startLLWithPipeline(int pipeline){
        if (!isBeingUsed) return;
        limelight1.start();limelight1.pipelineSwitch(pipeline);
        disconnectedTimer = new Timer();
    }
    //public int pipelineChange(Constants.AllianceColor allianceColor){int newPipe = changePipeline(limelight1.getLatestResult().getPipelineIndex(), allianceColor== Constants.AllianceColor.RED);limelight1.pipelineSwitch(newPipe);return newPipe;}

   public LLResultTypes.FiducialResult getResultForTag(int tagID) {
         if (!isBeingUsed) return null;
         for (LLResultTypes.FiducialResult fiduciary : fiducials ) {
             if (fiduciary.getFiducialId() == tagID) {
                 return fiduciary;
             }
         }
         return null;
   }

   public boolean isTagInFiducialResults(int tagID) {
       if (!isBeingUsed) return false;
       if (fiducials == null) return false;
       for (LLResultTypes.FiducialResult fiduciary : fiducials ) {
           if (fiduciary.getFiducialId() == tagID) {
               return true;
           }
       }
       return false;
   }
   public List<LLResultTypes.FiducialResult> getFiducials() {
       if (!isBeingUsed) return null;
       return fiducials;
   }
    public void LLUpdate() {
        // Must be called every loop for disconnected timer
        if (!isBeingUsed) return;
        result = limelight1.getLatestResult();
        if (result!=null && result.isValid()) {
            tx=result.getTx();
            ty= result.getTy();
            ta=result.getTa();
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiduciary : fiducials ) {
                tid = fiduciary.getFiducialId();
                dist = calculateDistanceFromRelativePose(fiduciary);
            }
            botpose = result.getBotpose_MT2();
            isDisconnected = false;
            disconnectedTimer.resetTimer();
        } else {
            if (disconnectedTimer.getElapsedTimeSeconds() > 0.6) {isDisconnected= true;}
        }
    }
    public double getLLScore() {
        if (!isBeingUsed) return Double.NaN;
        return Math.abs(tx);
    }

}

