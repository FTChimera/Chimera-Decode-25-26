package org.firstinspires.ftc.teamcode2.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode2.Auto.BLUE_AUTO_ARCHIVE;
import org.firstinspires.ftc.teamcode2.Auto.RED_AUTO_ARCHIVE;
import org.firstinspires.ftc.teamcode2.Systems.Constants;

@SuppressWarnings("SpellCheckingInspection")
@Autonomous(name = "Auto Start Position Test", group = "Test")
public class Auto_start_position extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private PathState pathState;

    public enum PathState {
        IDLE,
        LAUNCH,
        STARTPOSE
    }

    public final Pose startPose = new Pose(
            128.13,
            112,
            Math.toRadians(0)
    );

    public final Pose launchPose = new Pose(
            110,
            110,
            Math.toRadians(45)
    );

    public final Pose red_start_pose = RED_AUTO_ARCHIVE.startPose;

    public final Pose blue_start_pose = BLUE_AUTO_ARCHIVE.startPose;

    public Path launchPath;
    public Path startposePath;
    public Constants.Auto autoPose = Constants.Auto.RED_CLOSE;
    private Constants.AllianceColor allianceColor = Constants.AllianceColor.RED;

    public void buildPaths() {
        launchPath = new Path(
                new BezierLine(startPose, launchPose)
        );
        launchPath.setLinearHeadingInterpolation(
                startPose.getHeading(),
                launchPose.getHeading()
        );

        startposePath = new Path(
                new BezierLine(launchPose, allianceColor == Constants.AllianceColor.BLUE? blue_start_pose : red_start_pose)
        );
        startposePath.setLinearHeadingInterpolation(
                launchPose.getHeading(),
                (allianceColor == Constants.AllianceColor.BLUE? blue_start_pose : red_start_pose).getHeading()
        );
    }

    public void autonomousPathUpdate() {
        if (follower.isBusy()) return;

        switch (pathState) {
            case LAUNCH:
                follower.followPath(launchPath);
                setPathState(PathState.STARTPOSE);
                break;

            case STARTPOSE:
                follower.followPath(startposePath);
                setPathState(PathState.IDLE);
                break;
        }
    }

    public void setPathState(PathState state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createPedroFollower(hardwareMap);

        follower.setStartingPose(startPose);
        setPathState(PathState.LAUNCH);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Press A to switch alliance");
        telemetry.addData("Current Alliance", allianceColor);
        telemetry.update();
        if (gamepad1.aWasPressed()) {
            allianceColor = allianceColor.switchColors();
        }
    }

    @Override
    public void start() {
        pathTimer = new Timer();
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }
}