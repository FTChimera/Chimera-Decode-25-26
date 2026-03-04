package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.NewBot.Constants;

import java.util.List;

@SuppressWarnings("SpellCheckingInspection")
public class LimelightSystem {
    public enum Pipeline {
        APRIL_TAG(0);
        private final int pipelineIndex;
        Pipeline(int index) { this.pipelineIndex = index; }

        public int getIndex() { return pipelineIndex; }
    }
    public enum DISTANCE_METHOD {
        AREA, RELATIVE_POSE, TRIG
    }
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

    public static double calculateDistanceFromArea(double x) {
        return 1/x;
    }
    public static double calculateDistanceFromRelativePose(LLResultTypes.FiducialResult fiducial) {
        if (fiducial==null) return Double.NaN;
        Pose3D pose = fiducial.getRobotPoseTargetSpace();
        double x = pose.getPosition().x;
        double z = pose.getPosition().z;
        return Math.sqrt(x*x + z*z);
    }
    public double tx=0,ty=0,ta=0,tid=0, dist=0;
    public boolean isDisconnected;
    public Limelight3A limelight1;
    private Timer disconnectedTimer;
    public Pose3D botpose;
    public LLResult result;
    public final DISTANCE_METHOD distanceMethod = DISTANCE_METHOD.TRIG;
    private List<LLResultTypes.FiducialResult> fiducials;
    public void start(int pipeline) {startLLWithPipeline(pipeline);}
    public void start() {start(0);}
    public void start(Pipeline pipeline) {
        start(pipeline.getIndex());
    }
    private void startLLWithPipeline(int pipeline){
        if (!isBeingUsed) return;
        limelight1.start();limelight1.pipelineSwitch(pipeline);
        disconnectedTimer = new Timer();
    }
    //public int pipelineChange(Constants.AllianceColor allianceColor){int newPipe = changePipeline(limelight1.getLatestResult().getPipelineIndex(), allianceColor== Constants.AllianceColor.RED);limelight1.pipelineSwitch(newPipe);return newPipe;}

   public LLResultTypes.FiducialResult getResultForTag(int tagID) {
         if (!isBeingUsed) return null;
         if (fiducials == null) return null;
         for (LLResultTypes.FiducialResult fiduciary : fiducials ) {
             if (fiduciary.getFiducialId() == tagID) {
                 return fiduciary;
             }
         }
         return null;
   }
   public static double calculateDistanceFromTrig(LLResultTypes.FiducialResult result) {
        double ty = result.getTargetYDegrees();

        double angleToGoalRadians = Math.toRadians(Constants.LIMELIGHT_MOUNT_ANGLE_DEGREES + ty);
        return (Constants.GOAL_HEIGHT_INCHES - Constants.LIMELIGHT_LENS_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);
    }
   public double calculateDistance(LLResultTypes.FiducialResult result) {
         if (distanceMethod == DISTANCE_METHOD.AREA) {
              return calculateDistanceFromArea(result.getTargetArea());
         } else if (distanceMethod == DISTANCE_METHOD.RELATIVE_POSE){
              return calculateDistanceFromRelativePose(result);
         } else {
             return calculateDistanceFromTrig(result);
         }
   }
   public double findDistanceFromTag(int tagId) {
       if (!isBeingUsed) return Double.NaN;
       LLResultTypes.FiducialResult fiduciary = getResultForTag(tagId);
       if (fiduciary == null) return Double.NaN;
       return calculateDistance(fiduciary);
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
            fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiduciary : fiducials ) {
                tid = fiduciary.getFiducialId();
                dist = calculateDistance(fiduciary);
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

