package org.firstinspires.ftc.teamcode.NewBot;

import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;

import java.util.Arrays;

// Using PedroPathing's KalmanFilter implementation
@SuppressWarnings("SpellCheckingInspection")
public class KalmanAutoCorrectPedroLimelight {

    private final KalmanFilter kx;
    private final KalmanFilter ky;
    private final KalmanFilter kHeading;
    private Pose lastPose;

    // timestamps for simple prediction (not used for motion model here, but kept for potential extension)
    private long lastUpdateTimeMs = -1;

    public KalmanAutoCorrectPedroLimelight() {
        // Reasonable defaults: start at 0 with large initial variance
        double initVar = 1000.0; // large uncertainty initially
        double processNoise = 0.5; // small process noise
        double measNoise = 25.0; // measurement noise (tunable)

        KalmanFilterParameters px = new KalmanFilterParameters(processNoise, measNoise);
        KalmanFilterParameters py = new KalmanFilterParameters(processNoise, measNoise);
        KalmanFilterParameters pHeading = new KalmanFilterParameters(processNoise, measNoise);

        kx = new KalmanFilter(px);
        ky = new KalmanFilter(py);
        kHeading = new KalmanFilter(pHeading);

        // initialize with large variance / zero state
        kx.reset(0.0, initVar, 1.0);
        ky.reset(0.0, initVar, 1.0);
        kHeading.reset(0.0, initVar, 1.0);
    }

    /**
     * Update the filter using a Limelight-derived Pedro pose (if available).
     * This will update the filters with the measured pose. If the Limelight does not have a valid detection,
     * this method does nothing (we don't have a separate predict API on the provided KalmanFilter).
     *
     * @param limelight the LimelightSystem to query
     * @param allianceColor the alliance color (used by PedroDrive.getPedroPoseFromLL)
     */
    public void updateFromLimelight(LimelightSystem limelight, Constants.AllianceColor allianceColor, PedroDrive pedroDrive) {
        Pose measured = PedroDrive.getPedroPoseFromLL(limelight, allianceColor);
        if (measured == null) {
            // No measurement available
            return;
        }

        // The KalmanFilter API expects (updateData, updateProjection).
        kx.update(pedroDrive.getPose().getX() - lastPose.getX(), measured.getX());
        ky.update(pedroDrive.getPose().getY() - lastPose.getY(), measured.getY());
        kHeading.update(pedroDrive.getPose().getHeading() - lastPose.getHeading(), measured.getHeading());
        lastPose = pedroDrive.getPose();

        lastUpdateTimeMs = System.currentTimeMillis();
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

    public String[] output() {
        return new String[]{
                "Heading" + Arrays.toString(kHeading.output()),
                "X" + Arrays.toString(kx.output()),
                "Y" + Arrays.toString(ky.output())
        };}
}
