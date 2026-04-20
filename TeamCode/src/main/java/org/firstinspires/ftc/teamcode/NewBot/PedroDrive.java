package org.firstinspires.ftc.teamcode.NewBot;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
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
        Pose3D botpose = limelight.botpose;
        if (botpose == null) {
            return null; //Not valid botpose
        }
        return FTCCoordinates.INSTANCE.convertToPedro(
                        new Pose(
                                botpose.getPosition().x,
                                botpose.getPosition().z,
                                botpose.getOrientation().getYaw(AngleUnit.RADIANS)
                        )
                 );
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
    public Vector getVelocity() {
        return follower.getVelocity();
    }
    public void setTeleopDrive(double forward, double strafe, double rotation, boolean robotHeading) {
        follower.setTeleOpDrive(forward,strafe,rotation,robotHeading);
    }

}
