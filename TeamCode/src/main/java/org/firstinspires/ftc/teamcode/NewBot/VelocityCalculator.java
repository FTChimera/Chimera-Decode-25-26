package org.firstinspires.ftc.teamcode.NewBot;


@SuppressWarnings("SpellCheckingInspection")
public class VelocityCalculator {
    // Shot table

    public static final double minVelocityNewBot = 970, maxVelocityNewBot = 1500;
    public static final double[][] shotTableNewBot = {
            // { distance (1/ta), velocity }
            // fill in with tuned values- TODO - fix these based on mew launch angle + distance instead of target area
            {98, 1010},
            {138, 1060},
            {190, 1120},
            {230, 1140},
            {260, 1157},
            {310, 1220},
            {365, 1240},
            {425, 1330},
            {474, 1350},
            {535, 1400},
            {650, 1475}
    };
    public static VelocityCalculator NEWBOT = new VelocityCalculator(shotTableNewBot, minVelocityNewBot, maxVelocityNewBot);

    private final double[][] shotTable;
    private final double minVelocity, maxVelocity;

    public VelocityCalculator(double[][] table, double min, double max) {
        shotTable = table;
        minVelocity = min; maxVelocity = max;
    }
    public double calculateVelocity(double distance) {
        if (distance <= this.shotTable[0][0]) {
            return this.minVelocity; // edge case lower
        } else if (distance >= this.shotTable[this.shotTable.length-1][0]) {
            return this.maxVelocity; // edge case higher
        }
        for (int i = 0; i < this.shotTable.length - 1; i++) {
            double d0 = this.shotTable[i][0];
            double d1 = this.shotTable[i + 1][0];
            // dynamic scaling
            if (distance >= d0 && distance <= d1) {
                double r0 = this.shotTable[i][1];
                double r1 = this.shotTable[i + 1][1];
                double t = (distance - d0) / (d1 - d0);
                return r0 + t * (r1 - r0);
            }
        }
        return Double.NaN; // Should never reach here
    }

}
