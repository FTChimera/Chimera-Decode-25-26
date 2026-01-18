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

@Autonomous(name = "Red Auto", group = "Pedro Auto", preselectTeleOp = "Pedro_TeleOp")
public class RED_AUTO extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private PathState pathState;
    private  AutoHelper autoHelper;

    public enum PathState {
        IDLE,
        LAUNCH,
        SET_1,
        INTAKE1,
        LAUNCH_1,
        SET2,
        INTAKE2,
        EMPTY_GATE_0,
        BACK_LAUNCH,
        SET3,
        INTAKE3,
        BACK_LAUNCH_1,
        END
    }

    public final Pose startPose = new Pose(
            123.5,
            122.8,
            Math.toRadians(37.5)
    );

    public final Pose launchPose = new Pose(
            110,
            110,
            Math.toRadians(45)
    );

    public final Pose set_1Pose = new Pose(
            102,
            84,
            Math.toRadians(0)
    );

    public final Pose intake1Pose = new Pose(
            129,
            84,
            0
    );

    public final Pose set2Pose = new Pose(
            102,
            60,
            Math.toRadians(0)
    );

    public final Pose intake2Pose = new Pose(
            125,
            60,
            0
    );

    public final Pose empty_gate_0Pose = new Pose(
            131,
            72,
            Math.toRadians(90)
    );

    public final Pose empty_gate_0ControlPoint1 = new Pose(
            110,
            72,
            0
    );

    public final Pose empty_gate_0ControlPoint2 = new Pose(
            120,
            72,
            0
    );

    public final Pose back_launchPose = new Pose(
            84,
            12,
            Math.toRadians(65.6)
    );

    public final Pose set3Pose = new Pose(
            102,
            36,
            Math.toRadians(0)
    );

    public final Pose intake3Pose = new Pose(
            132,
            36,
            0
    );

    public static final Pose endPose = new Pose(
            84,
            36,
            Math.toRadians(65.6)
    );

    public Path launchPath;
    public Path set_1Path;
    public Path intake1Path;
    public Path launchPath_1;
    public Path set2Path;
    public Path intake2Path;
    public Path empty_gate_0Path;
    public Path back_launchPath;
    public Path set3Path;
    public Path intake3Path;
    public Path back_launchPath_1;
    public Path endPath;

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

        empty_gate_0Path = new Path(
                new BezierCurve(intake2Pose,
                        empty_gate_0ControlPoint1,
                        empty_gate_0ControlPoint2,
                        empty_gate_0Pose)
        );
        empty_gate_0Path.setLinearHeadingInterpolation(
                intake2Pose.getHeading(),
                empty_gate_0Pose.getHeading()
        );

        back_launchPath = new Path(
                new BezierCurve(empty_gate_0Pose,
                        empty_gate_0ControlPoint2,
                        back_launchPose)
        );
        back_launchPath.setLinearHeadingInterpolation(
                empty_gate_0Pose.getHeading(),
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
            case LAUNCH:
                follower.followPath(launchPath);
                if (autoHelper.runLauncherSequence(false, 3)) {
                    setPathState(PathState.SET_1);
                    break;
                }

            case SET_1:
                follower.followPath(set_1Path);
                autoHelper.Intake();
                setPathState(PathState.INTAKE1);
                break;

            case INTAKE1:
                follower.followPath(intake1Path);
                autoHelper.IntakeStop();
                setPathState(PathState.LAUNCH_1);
                break;

            case LAUNCH_1:
                follower.followPath(launchPath_1);
                if (autoHelper.runLauncherSequence(false, 3)) {
                    setPathState(PathState.SET2);
                    break;
                }

            case SET2:
                follower.followPath(set2Path);
                autoHelper.Intake();
                setPathState(PathState.INTAKE2);
                break;

            case INTAKE2:
                follower.followPath(intake2Path);
                autoHelper.IntakeStop();
                setPathState(PathState.EMPTY_GATE_0);
                break;

            case EMPTY_GATE_0:
                follower.followPath(empty_gate_0Path);
                setPathState(PathState.BACK_LAUNCH);
                break;

            case BACK_LAUNCH:
                follower.followPath(back_launchPath);
                if (autoHelper.runLauncherSequence(true, 3)) {
                    setPathState(PathState.SET3);
                    break;
                }

            case SET3:
                follower.followPath(set3Path);
                autoHelper.Intake();
                setPathState(PathState.INTAKE3);
                break;

            case INTAKE3:
                follower.followPath(intake3Path);
                autoHelper.IntakeStop();
                setPathState(PathState.BACK_LAUNCH_1);
                break;

            case BACK_LAUNCH_1:
                follower.followPath(back_launchPath_1);
                if (autoHelper.runLauncherSequence(true, 3)) {
                    setPathState(PathState.END);
                    break;
                }

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
        autoHelper = new AutoHelper(hardwareMap);
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