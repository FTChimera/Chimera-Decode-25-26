package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;

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

    public double CalculateDistanceCurve(double TargetArea) {
        // Calculates the Distance based on a formula from the Target Area.
        return TargetArea;
    }

    public class LimelightResult {
        public double tx=-1,ty=-1,ta=-1,tid=-1, dist=-1;

        public Limelight3A limelight1;

        public double getTa() {return ta;}
        public double getId() {return tid;}
        public double[] getTxy() {return new double[]{tx, ty};}

    }
}

