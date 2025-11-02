package org.firstinspires.ftc.teamcode.Vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Consts;
import org.firstinspires.ftc.teamcode.pedroAuto.Constants;

@TeleOp
public class LLOrientation extends LinearOpMode {
    public Pose startingPose;
    public static Consts.AllianceColor allianceColor = Consts.AllianceColor.RED;
    private Follower follower = Constants.createFollower(hardwareMap);
    public Pose curPose = follower.getPose();
    public LLToolkit.ChimeraLL limelight = new LLToolkit.ChimeraLL();
    public double findAngleToRotate() {
        // return allianceColor==Consts.AllianceColor.RED? Math.atan2( (Consts.Y_Coordinate_Red_Goal - curPose.getY()), (Consts.X_Coordinate_Red_Goal - curPose.getX()) ):Math.atan2( (Consts.Y_Coordinate_Blue_Goal - curPose.getY()), (Consts.X_Coordinate_Blue_Goal - curPose.getX()) );
        return Math.atan2( (limelight.dist), limelight.tx ); // Maybe? multiply tx by value and see accuracy of distance.
    }
    public void rotate(double angle_Radians) {follower.setPose(new Pose(curPose.getX(), curPose.getY(), angle_Radians));}
    @Override
    public void runOpMode() {
        limelight.setDevice(hardwareMap.get(Limelight3A.class, "limelight"));
        startingPose = allianceColor==Consts.AllianceColor.RED? Consts.RED_STARTING_POSE : Consts.BLUE_STARTING_POSE;follower.setStartingPose(startingPose);follower.update();
        waitForStart();limelight.startLLWithPipeline(0);follower.startTeleopDrive();
        while (opModeIsActive()) {
            limelight.LLUpdate();follower.update();
            if (gamepad2.a) {
                rotate(findAngleToRotate());
            }
        }
    }
}
