package org.firstinspires.ftc.teamcode2.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode2.pedroPathing.Constants;

@Autonomous(name = "Blue Close Auto", group = "Pedro Auto")
public class Blue_Close extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private PathState pathState;

    public enum PathState {
        IDLE,
        LAUNCH,
        SET1,
        INTAKE1,
        LAUNCH_1,
        SET2,
        INTAKE2,
        LAUNCHSET2,
        SET3,
        INTAKE3,
        LAUNCH_2,
        END
    }

    public static final Pose startPose = new Pose(
            20.5,
            122.8,
            Math.toRadians(143)
    );

    public static final Pose launchPose = new Pose(
            36,
            108,
            Math.toRadians(135)
    );

    public static final Pose set1Pose = new Pose(
            48,
            84,
            Math.toRadians(180)
    );

    public static final Pose intake1Pose = new Pose(
            18,
            84,
            0
    );

    public static final Pose set2Pose = new Pose(
            48,
            60,
            Math.toRadians(180)
    );

    public static final Pose intake2Pose = new Pose(
            18,
            60,
            0
    );

    public static final Pose launchset2ControlPoint1 = new Pose(
            36,
            60,
            0
    );

    public static final Pose set3Pose = new Pose(
            48,
            36,
            Math.toRadians(180)
    );

    public static final Pose intake3Pose = new Pose(
            18,
            36,
            0
    );

    public static final Pose endPose = new Pose(
            36,
            72,
            Math.toRadians(180)
    );

    public Path launchPath;
    public Path set1Path;
    public Path intake1Path;
    public Path launchPath_1;
    public Path set2Path;
    public Path intake2Path;
    public Path launchset2Path;
    public Path set3Path;
    public Path intake3Path;
    public Path launchPath_2;
    public Path endPath;

    public void buildPaths() {
        launchPath = new Path(
                new BezierLine(startPose, launchPose)
        );
        launchPath.setLinearHeadingInterpolation(
                startPose.getHeading(),
                launchPose.getHeading()
        );

        set1Path = new Path(
                new BezierLine(launchPose, set1Pose)
        );
        set1Path.setLinearHeadingInterpolation(
                launchPose.getHeading(),
                set1Pose.getHeading()
        );

        intake1Path = new Path(
                new BezierLine(set1Pose, intake1Pose)
        );
        intake1Path.setLinearHeadingInterpolation(
                set1Pose.getHeading(),
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

        launchset2Path = new Path(
                new BezierCurve(intake2Pose,
                        launchset2ControlPoint1,
                        launchPose)
        );
        launchset2Path.setLinearHeadingInterpolation(
                intake2Pose.getHeading(),
                launchPose.getHeading()
        );

        set3Path = new Path(
                new BezierLine(launchPose, set3Pose)
        );
        set3Path.setLinearHeadingInterpolation(
                launchPose.getHeading(),
                set3Pose.getHeading()
        );

        intake3Path = new Path(
                new BezierLine(set3Pose, intake3Pose)
        );
        intake3Path.setLinearHeadingInterpolation(
                set3Pose.getHeading(),
                intake3Pose.getHeading()
        );

        launchPath_2 = new Path(
                new BezierLine(intake3Pose, launchPose)
        );
        launchPath_2.setLinearHeadingInterpolation(
                intake3Pose.getHeading(),
                launchPose.getHeading()
        );

        endPath = new Path(
                new BezierLine(launchPose, endPose)
        );
        endPath.setLinearHeadingInterpolation(
                launchPose.getHeading(),
                endPose.getHeading()
        );
    }

    public void autonomousPathUpdate() {
        if (follower.isBusy()) return;

        switch (pathState) {
            case LAUNCH:
                follower.followPath(launchPath);
                setPathState(PathState.SET1);
                break;

            case SET1:
                follower.followPath(set1Path);
                setPathState(PathState.INTAKE1);
                break;

            case INTAKE1:
                follower.followPath(intake1Path);
                setPathState(PathState.LAUNCH_1);
                break;

            case LAUNCH_1:
                follower.followPath(launchPath_1);
                setPathState(PathState.SET2);
                break;

            case SET2:
                follower.followPath(set2Path);
                setPathState(PathState.INTAKE2);
                break;

            case INTAKE2:
                follower.followPath(intake2Path);
                setPathState(PathState.LAUNCHSET2);
                break;

            case LAUNCHSET2:
                follower.followPath(launchset2Path);
                setPathState(PathState.SET3);
                break;

            case SET3:
                follower.followPath(set3Path);
                setPathState(PathState.INTAKE3);
                break;

            case INTAKE3:
                follower.followPath(intake3Path);
                setPathState(PathState.LAUNCH_2);
                break;

            case LAUNCH_2:
                follower.followPath(launchPath_2);
                setPathState(PathState.END);
                break;

            case END:
                follower.followPath(endPath);
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
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        setPathState(PathState.LAUNCH);
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