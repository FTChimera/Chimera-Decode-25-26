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
    public static Consts.AllianceColor allianceColor = Consts.AllianceColor.RED;
    private Follower follower = Constants.createFollower(hardwareMap);
    Pose curPose = follower.getPose();
    public LLToolkit.ChimeraLL limelight = new LLToolkit.ChimeraLL();
    public void rotate(double angle_Radians) {
        follower.setPose(new Pose(curPose.getX(), curPose.getY(), angle_Radians));
    }
    @Override
    public void runOpMode() {
        limelight.setDevice(hardwareMap.get(Limelight3A.class, "limelight"));
        waitForStart();limelight.startLLWithPipeline(0);
        while (opModeIsActive()) {
            limelight.LLUpdate();

        }
    }
}
