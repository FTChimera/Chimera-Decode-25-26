package org.firstinspires.ftc.teamcode.biobuzzTest;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NewBot.Constants;

@TeleOp(name = "RED TELEOP", group = "Test")
public class RED_TELEOP extends NewBotTeleAbstract {

    public RED_TELEOP() {
        super(Constants.AllianceColor.RED, true);
    }

    @Override
    Pose getStartingPose() {
        return Constants.CHIMERA_TESTING_POSE;
    }

}
