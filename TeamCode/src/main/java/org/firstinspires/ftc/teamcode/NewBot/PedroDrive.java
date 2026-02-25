package org.firstinspires.ftc.teamcode.NewBot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;

@SuppressWarnings("SpellCheckingInspection")
public class PedroDrive {
    private Follower follower;
    private Pose lastPose;
    public PedroDrive(HardwareMap hwMap, Pose initialPose) {
        follower = Constants.createPedroFollower(hwMap);
        follower.setStartingPose(initialPose);
        lastPose = initialPose;
    }

    public static double getPedroHeadingFromLL(LimelightSystem limelight, Constants.AllianceColor allianceColor) {
        double tx = limelight.tx;
        double headingOffset = allianceColor == Constants.AllianceColor.RED ? 45 : 135;
        return (tx + headingOffset) % 360;
    }

    public double getLLHeadingFromPedro(Constants.AllianceColor allianceColor) {
        double headingOffset = allianceColor == Constants.AllianceColor.RED ? 45 : 135;
        double heading = lastPose.getHeading();
        return heading - headingOffset;
    }

    private Pose getCurrentPose() {
        return follower.getPose();
    }

    public Pose getPose() {
        return lastPose;
    }

    public void update() {
        // Update last known pose
        lastPose = getCurrentPose();

        // Update follower (handles odometry and motor control)
        follower.update();
    }

    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }

    public void correctPose(Pose pose) {
        follower.setPose(pose);
    }

    public void resetHeading(double newHeading) {
        follower.setPose(getCurrentPose().setHeading(newHeading));
    }

    public double getAngularVelocity() {
        return follower.getAngularVelocity();
    }

    public Vector getAcceleration() {
        return follower.getAcceleration();
    }

}
