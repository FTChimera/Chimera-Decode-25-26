package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;

public class LLToolkit {

    public int changePipeline(int curPipe, boolean isRed) {
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

    public double calculateDistanceCurve(double x) {
        // Calculate the exponent term (x / 6.633929)^7.298902
        double exponentTerm = Math.pow(x / 6.633929, 7.298902);

        // Calculate the denominator (1 + exponentTerm)
        double denominator = 1 + exponentTerm;

        // Calculate the numerator (84.12366 - 46.68694)
        double numerator = 84.12366 - 46.68694;

        // Perform the division
        double divisionResult = numerator / denominator;

        // Calculate the final value of y

        return 46.68694 + divisionResult;
    }
    public class ChimeraLL {
        public double tx=-1.0,ty=-1.0,ta=-1.0,tid=-1.0, dist=-1.0;

        public Limelight3A limelight1;

        public void setDevice(Limelight3A device) {limelight1=device;}
        public void startLL(){limelight1.start();}
        public double[] getLatest(){return new double[]{tx,ty,ta,tid,dist};}
        public void LLUpdate() {
            LLResult result = limelight1.getLatestResult();
            if (result!=null && result.isValid()) {
                tx=result.getTx();
                ty= result.getTy();
                ta=result.getTa();
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiduciary : fiducials ) {tid = fiduciary.getFiducialId();}
                dist = calculateDistanceCurve(ta);
            }
        }
    }
}

