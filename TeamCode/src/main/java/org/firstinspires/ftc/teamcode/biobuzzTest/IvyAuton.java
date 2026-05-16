package org.firstinspires.ftc.teamcode.biobuzzTest;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.NewBot.Constants;

@Autonomous(name = "IVY AUTON", group = "0: BIOBUZZ TEST")
public class IvyAuton extends OpMode {
    Follower follower;
    DcMotor intake;
    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.createPedroFollower(hardwareMap);
        follower.setStartingPose(new Pose(36, 12, Math.toRadians(180)));
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
    }

    @Override public void start() {
        Scheduler.schedule(autoRoutine());
    }
    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }

    public Command autoRoutine() {
        Command intakeFullPower = instant(() -> intake.setPower(1));
        Command intakeStop = instant(() -> intake.setPower(0));
        Path path = new Path(new com.pedropathing.geometry.BezierLine(
                new Pose(36, 12, Math.toRadians(180)),
                new Pose(36, 36, Math.toRadians(180))
        ));
        path.setConstantHeadingInterpolation(Math.toRadians(180));
        Command followPath = follow(follower, new PathChain(path), true);
        return sequential(
                intakeFullPower,
                followPath,
                waitMs(1000),
                intakeStop
        );
    }
}
