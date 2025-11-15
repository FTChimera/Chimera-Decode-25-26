package org.firstinspires.ftc.teamcode.Vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Systems.Consts;
import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode.pedroAuto.Constants;

@TeleOp
public class LLOrientation extends LinearOpMode {
    public static Consts.AllianceColor allianceColor = Consts.AllianceColor.RED;
    private Follower follower;
    public LimelightSystem.ChimeraLL limelight = new LimelightSystem.ChimeraLL();
    public double findAngleToRotate() {
        // return allianceColor==Consts.AllianceColor.RED? Math.atan2( (Consts.Y_Coordinate_Red_Goal - curPose.getY()), (Consts.X_Coordinate_Red_Goal - curPose.getX()) ):Math.atan2( (Consts.Y_Coordinate_Blue_Goal - curPose.getY()), (Consts.X_Coordinate_Blue_Goal - curPose.getX()) );
        return Math.toRadians(limelight.tx);
    }
    public Path getPath(double amt) {
        Pose pose = follower.getPose();
        double heading = pose.getHeading();
        double RobotCentric_X = Math.sin(heading);
        double RobotCentric_Y = Math.cos(heading);
        Pose endPose = new Pose(pose.getX() + RobotCentric_X*amt, pose.getY() + RobotCentric_Y*amt, heading);
        Path path = new Path(new BezierLine(pose, endPose));path.setLinearHeadingInterpolation(heading,heading);
        return path;
    }
    public void rotate(double angle_Radians) {follower.turn(Math.abs(angle_Radians), angle_Radians/Math.abs(angle_Radians)==-1);follower.update();}
    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        limelight.setDevice(hardwareMap.get(Limelight3A.class, "limelight"));
        // startingPose = allianceColor==Consts.AllianceColor.RED? Consts.RED_STARTING_POSE : Consts.BLUE_STARTING_POSE;follower.setStartingPose(startingPose);follower.update();
        waitForStart();limelight.startLLWithPipeline(allianceColor==Consts.AllianceColor.RED?4:5);follower.startTeleopDrive();


        while (opModeIsActive()) {
            limelight.LLUpdate();
            telemetry.addData("Tx (radians)", Math.toRadians(limelight.tx));
            telemetry.addData("Tx (degrees)", limelight.tx);
            telemetry.addData("Gamepad2.a", "not pressed");
            if (gamepad2.a) {
                int step = 0;
                if (gamepad2.dpad_right) {step=step+1;}
                telemetry.addData("Gamepad2.a", "pressed");
                // FIRST, rotate to face the goal
                if (step==0) {
                    rotate(findAngleToRotate() * Consts.LAUNCHER_GOALTAG_ANGLE_SCALE);
                }
                // THEN, you need to move so that the distance is good for launching.
                if (step==1){
                    follower.followPath(getPath(limelight.dist - Consts.LAUNCHER_GOALTAG_OFFSET));
                    follower.update();
                }
            } else {
                rotate(0);
                follower.followPath(new Path());
            }

            telemetry.update();
        }
    }
}
