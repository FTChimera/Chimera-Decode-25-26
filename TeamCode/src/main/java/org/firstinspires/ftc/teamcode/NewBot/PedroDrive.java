package org.firstinspires.ftc.teamcode.NewBot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
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

    public static Pose getPedroPoseFromLL(LimelightSystem limelight, Constants.AllianceColor allianceColor) {
        int tagID = allianceColor == Constants.AllianceColor.RED ? 24 : 20;
        Pose AllianceGoalPose = allianceColor == Constants.AllianceColor.RED ? Constants.RED_GOAL : Constants.BLUE_GOAL;
        LLResultTypes.FiducialResult fiducial = limelight.getResultForTag(tagID);
        if (fiducial == null) {
            return null; // No valid fiducial result, cannot determine pose
        }
        Pose3D Pose3d = fiducial.getRobotPoseTargetSpace();
        double x = Pose3d.getPosition().x + AllianceGoalPose.getX();
        double y = Pose3d.getPosition().z + AllianceGoalPose.getY(); // LL's z is our y
        double heading = Pose3d.getOrientation().getYaw() + AllianceGoalPose.getHeading();
        return new Pose(x, y, Math.toRadians(heading));
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
