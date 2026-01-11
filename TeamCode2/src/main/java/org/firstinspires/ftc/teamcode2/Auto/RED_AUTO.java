package org.firstinspires.ftc.teamcode2.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode2.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode2.Systems.Consts;

@Autonomous(name = "Red Auto", group = "Pedro Auto")
public class RED_AUTO extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private final Pose startPose = new Pose(
            128.13,
            110.81,
            Math.toRadians(90)
    );

    private final Pose launchPose = new Pose(
            110,
            110,
            Math.toRadians(45)
    );

    private final Pose set_1Pose = new Pose(
            108,
            84,
            Math.toRadians(0)
    );

    private final Pose intake1Pose = new Pose(
            134,
            84,
            0
    );

    private final Pose set2Pose = new Pose(
            108,
            60,
            Math.toRadians(0)
    );

    private final Pose intake2Pose = new Pose(
            136,
            60,
            0
    );

    private final Pose back_launchPose = new Pose(
            84,
            12,
            Math.toRadians(50)
    );

    private final Pose set3Pose = new Pose(
            108,
            36,
            Math.toRadians(0)
    );

    private final Pose intake3Pose = new Pose(
            136,
            36,
            0
    );

    private final Pose endPose = new Pose(
            84,
            36,
            Math.toRadians(50)
    );

    private Path launchPath;
    private Path set_1Path;
    private Path intake1Path;
    private Path launchPath_1;
    private Path set2Path;
    private Path intake2Path;
    private Path back_launchPath;
    private Path set3Path;
    private Path intake3Path;
    private Path back_launchPath_1;
    private Path endPath;

    public void buildPaths() {
        launchPath = new Path(
                new BezierLine(startPose, launchPose)
        );
        launchPath.setLinearHeadingInterpolation(
                startPose.getHeading(),
                launchPose.getHeading()
        );

        set_1Path = new Path(
                new BezierLine(launchPose, set_1Pose)
        );
        set_1Path.setLinearHeadingInterpolation(
                launchPose.getHeading(),
                set_1Pose.getHeading()
        );

        intake1Path = new Path(
                new BezierLine(set_1Pose, intake1Pose)
        );
        intake1Path.setLinearHeadingInterpolation(
                set_1Pose.getHeading(),
                intake1Pose.getHeading()
        );

        launchPath_1 = new Path(
                new BezierLine(intake1Pose, launchPose)
        );
        launchPath_1.setLinearHeadingInterpolation(
                intake1Pose.getHeading(),
                launchPose.getHeading()
        );

        set2Path = new Path(
                new BezierLine(launchPose, set2Pose)
        );
        set2Path.setLinearHeadingInterpolation(
                launchPose.getHeading(),
                set2Pose.getHeading()
        );

        intake2Path = new Path(
                new BezierLine(set2Pose, intake2Pose)
        );
        intake2Path.setLinearHeadingInterpolation(
                set2Pose.getHeading(),
                intake2Pose.getHeading()
        );

        back_launchPath = new Path(
                new BezierLine(intake2Pose, back_launchPose)
        );
        back_launchPath.setLinearHeadingInterpolation(
                intake2Pose.getHeading(),
                back_launchPose.getHeading()
        );

        set3Path = new Path(
                new BezierLine(back_launchPose, set3Pose)
        );
        set3Path.setLinearHeadingInterpolation(
                back_launchPose.getHeading(),
                set3Pose.getHeading()
        );

        intake3Path = new Path(
                new BezierLine(set3Pose, intake3Pose)
        );
        intake3Path.setLinearHeadingInterpolation(
                set3Pose.getHeading(),
                intake3Pose.getHeading()
        );

        back_launchPath_1 = new Path(
                new BezierLine(intake3Pose, back_launchPose)
        );
        back_launchPath_1.setLinearHeadingInterpolation(
                intake3Pose.getHeading(),
                back_launchPose.getHeading()
        );

        endPath = new Path(
                new BezierLine(back_launchPose, endPose)
        );
        endPath.setLinearHeadingInterpolation(
                back_launchPose.getHeading(),
                endPose.getHeading()
        );
    }

    public void autonomousPathUpdate() {
        if (follower.isBusy()) return;

        switch (pathState) {
            case 0:
                follower.followPath(launchPath);
                setPathState(1);
                break;

            case 1:
                follower.followPath(set_1Path);
                setPathState(2);
                break;

            case 2:
                follower.followPath(intake1Path);
                setPathState(3);
                break;

            case 3:
                follower.followPath(launchPath_1);
                setPathState(4);
                break;

            case 4:
                follower.followPath(set2Path);
                setPathState(5);
                break;

            case 5:
                follower.followPath(intake2Path);
                setPathState(6);
                break;

            case 6:
                follower.followPath(back_launchPath);
                setPathState(7);
                break;

            case 7:
                follower.followPath(set3Path);
                setPathState(8);
                break;

            case 8:
                follower.followPath(intake3Path);
                setPathState(9);
                break;

            case 9:
                follower.followPath(back_launchPath_1);
                setPathState(10);
                break;

            case 10:
                follower.followPath(endPath);
                setPathState(11);
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
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