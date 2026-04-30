package org.firstinspires.ftc.teamcode.NewBot;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;

@SuppressWarnings("SpellCheckingInspection")
public class GoBildaPinpoint implements Localizer {
    PinpointLocalizer pinpoint;
    public boolean poseUpdated;
    Limelight3A limelight;

    public GoBildaPinpoint(HardwareMap hwMap) {
        pinpoint = new PinpointLocalizer(hwMap, Constants.pedroLocalizerConstants);
        limelight = hwMap.get(Limelight3A.class, "limelight");
    }

    public void start() {
        limelight.start();
        limelight.pipelineSwitch(LimelightSystem.Pipeline.APRIL_TAG.getIndex());
    }

    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint.getPinpoint();
    }
    public Limelight3A getLimelight() {
        return limelight;
    }

    /**
     * This returns the current pose estimate from the Localizer.
     *
     * @return returns the pose as a Pose object.
     */
    @Override
    public Pose getPose() {
        return pinpoint.getPose();
    }

    /**
     * This returns the current velocity estimate from the Localizer.
     *
     * @return returns the velocity as a Pose object.
     */
    @Override
    public Pose getVelocity() {
        return pinpoint.getVelocity();
    }

    /**
     * This returns the current velocity estimate from the Localizer as a Vector.
     *
     * @return returns the velocity as a Vector.
     */
    @Override
    public Vector getVelocityVector() {
        return pinpoint.getVelocityVector();
    }

    /**
     * This sets the start pose of the Localizer. Changing the start pose should move the robot as if
     * all its previous movements were displacing it from its new start pose.
     *
     * @param setStart the new start pose
     */
    @Override
    public void setStartPose(Pose setStart) {
        pinpoint.setStartPose(setStart);
    }

    /**
     * This sets the current pose estimate of the Localizer. Changing this should just change the
     * robot's current pose estimate, not anything to do with the start pose.
     *
     * @param setPose the new current pose estimate
     */
    @Override
    public void setPose(Pose setPose) {
        pinpoint.setPose(setPose);
    }

    /**
     * This calls an update to the Localizer, updating the current pose estimate and current velocity
     * estimate.
     */
    @Override
    public void update() {
        pinpoint.update();
        limelight.updateRobotOrientation(pinpoint.getPose().getHeading()); // Todo check offsets
        try {
            Pose3D botPose = limelight.getLatestResult().getBotpose_MT2();
            double pedroX = botPose.getPosition().x - 72;
            double pedroY = botPose.getPosition().z - 72;
            double pedroHeading = botPose.getOrientation().getYaw(AngleUnit.RADIANS); // todo offsets
            pinpoint.setPose(new Pose(pedroX, pedroY, pedroHeading));
            poseUpdated = true;
        } catch (Throwable e) {
            poseUpdated = false;
        }
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    @Override
    public double getTotalHeading() {
        return pinpoint.getTotalHeading();
    }

    /**
     * This returns the multiplier applied to forward movement measurement to convert from encoder
     * ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    @Override
    public double getForwardMultiplier() {
        return pinpoint.getForwardMultiplier();
    }

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * encoder ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    @Override
    public double getLateralMultiplier() {
        return pinpoint.getLateralMultiplier();
    }

    /**
     * This returns the multiplier applied to turning movement measurement to convert from encoder
     * ticks to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    @Override
    public double getTurningMultiplier() {
        return pinpoint.getTurningMultiplier();
    }

    /**
     * This resets the IMU of the localizer, if applicable.
     */
    @Override
    public void resetIMU() {
        pinpoint.resetIMU();
    }

    /**
     * This is overridden to return the IMU's heading estimate, if there is one.
     *
     * @return returns the IMU's heading estimate if it exists
     */
    @Override
    public double getIMUHeading() {
        return pinpoint.getIMUHeading();
    }

    /**
     * This returns whether if any component of robot's position is NaN.
     *
     * @return returns if any component of the robot's position is NaN
     */
    @Override
    public boolean isNAN() {
        return pinpoint.isNAN();
    }
}
