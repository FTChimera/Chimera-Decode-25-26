package org.firstinspires.ftc.teamcode.NewBot;


import com.pedropathing.control.LowPassFilter;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;

import java.util.Arrays;

// Using PedroPathing's LowPassFilter implementation
@SuppressWarnings("SpellCheckingInspection")
public class LPF_Corrector {
    public boolean isMeasured;
    private LowPassFilter kx, ky, kHeading;
    private Pose lastPose;
    private double lastUpdateTimeMs;

    public LPF_Corrector() {
        double alpha = 0.3;;
        kx = new LowPassFilter(alpha);
        ky = new LowPassFilter(alpha);
        kHeading = new LowPassFilter(alpha);
    }

    /**
     * Update the filter using a Limelight-derived Pedro pose (if available).
     * This will update the filters with the measured pose. If the Limelight does not have a valid detection,
     * this method does nothing (we don't have a separate predict API on the provided LowPassFilter).
     *
     * @param limelight the LimelightSystem to query
     * @param allianceColor the alliance color (used by PedroDrive.getPedroPoseFromLL)
     */
    public void updateFromLimelight(LimelightSystem limelight, Constants.AllianceColor allianceColor, PedroDrive pedroDrive) {
        Pose measured = PedroDrive.getPedroPoseFromLL(limelight, allianceColor);
        if (measured == null) {
            // No measurement available
            isMeasured = false;
            return;
        }

        // The LowPassFilter API expects (updateData, updateProjection).
        kx.update(pedroDrive.getPose().getX() - lastPose.getX(), 0);
        ky.update(pedroDrive.getPose().getY() - lastPose.getY(), 0);
        kHeading.update(pedroDrive.getPose().getHeading() - lastPose.getHeading(), 0);
        lastPose = pedroDrive.getPose();

        lastUpdateTimeMs = System.currentTimeMillis();
        isMeasured = true;
    }

    /**
     * Get the current estimated pose from the filter.
     * @return fused Pose (x, y, heading in radians)
     */
    public Pose getEstimatedPose() {
        return new Pose(kx.getState(), ky.getState(), kHeading.getState());
    }

    /**
     * Reset the internal filters to a provided pose.
     * @param pose the pose to reset to
     */
    public void resetToPose(Pose pose) {
        lastPose = pose;
        // small variance after reset
        double smallVar = 1.0;
        kx.reset(pose.getX(), smallVar, 1.0);
        ky.reset(pose.getY(), smallVar, 1.0);
        kHeading.reset(pose.getHeading(), smallVar, 1.0);
        lastUpdateTimeMs = System.currentTimeMillis();
    }

    /**
     * Apply the current estimated pose back to a PedroDrive instance to correct its pose.
     * @param drive the PedroDrive whose pose should be corrected
     */
    public void applyCorrectionToDrive(PedroDrive drive) {
        if (drive == null) return;
        Pose est = getEstimatedPose();
        if (est != null) {
            drive.correctPose(est);
        }
    }

}
