package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Systems.Consts.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.Systems.Consts.RED_GOAL;

import com.pedropathing.geometry.Pose;

/**
 * Computes flywheel velocity based on Pedro pose distance to the active alliance goal.
 * Assumes:
 *  - Pedro Pose units are inches
 *  - Heading is in radians (Pedro standard)
 *  - RED_GOAL and BLUE_GOAL are defined in bottom-left field coordinates
 */
public class VelocityCalculator {
    // Field coordinates of the goal (units must match Pose)

    public enum Type { POSE }
    private Type type;
    private Pose curPose;

    // shotTable format: { distance, flywheelVelocity }
    // Units must match Pose units and motor velocity units
    public final double[][] shotTable = {
            // FILL IN USING TESTER
            // TEST VALUES
            {10, 2}
    };

    // CONSTRUCTOR
    public VelocityCalculator(Pose startPose, Type calcType) {
        this.curPose = startPose;
        this.type = calcType;
    }

    public void update(Pose currentPose) {
        this.curPose = currentPose;
    }
    private Pose goalPose() {
        double heading = curPose.getHeading();

        // Normalize heading to [0, 2π)
        while (heading < 0) heading += 2 * Math.PI;
        while (heading >= 2 * Math.PI) heading -= 2 * Math.PI;

        // Facing generally "up" the field (toward red side)
        if (heading > Math.PI / 2 && heading < 3 * Math.PI / 2) {
            return RED_GOAL;
        }
        return BLUE_GOAL;
    }

    public double getVelocity() {
        if (shotTable.length == 0) {
            return Double.NaN;
        }
        if (curPose == null) {
            return Double.NaN;
        }
        if (type == Type.POSE) {
            double dx = goalPose().getX() - curPose.getX();
            double dy = goalPose().getY() - curPose.getY();
            double distance = Math.hypot(dx, dy);
            if (distance <= shotTable[0][0]) {
                return shotTable[0][1];
            } else if (distance >= shotTable[shotTable.length-1][0]) {
                return shotTable[shotTable.length-1][1];
            }
            for (int i = 0; i < shotTable.length - 1; i++) {
                double d0 = shotTable[i][0];
                double d1 = shotTable[i + 1][0];

                if (distance >= d0 && distance <= d1) {
                    double r0 = shotTable[i][1];
                    double r1 = shotTable[i + 1][1];
                    double t = (distance - d0) / (d1 - d0);
                    return r0 + t * (r1 - r0);
                }
            }
        }
        return Double.NaN;
    }
}
