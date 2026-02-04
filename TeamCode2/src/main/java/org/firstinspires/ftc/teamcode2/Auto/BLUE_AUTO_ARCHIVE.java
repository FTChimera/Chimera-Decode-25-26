package org.firstinspires.ftc.teamcode2.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode2.Systems.Constants;

@SuppressWarnings("SpellCheckingInspection")
@Disabled
@Autonomous(name = "BLUE ARCHIVE", group = "Pedro Auto", preselectTeleOp = "Pedro_TeleOp")
public class BLUE_AUTO_ARCHIVE extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private PathState pathState;
    private AutoHelper autoHelper;

    public enum PathState {
        IDLE,
        LAUNCH,
        SET1,
        INTAKE1,
        LAUNCH_1,
        SET2,
        INTAKE2,
        EMPTY_GATE,
        BACK_LAUNCH,
        SET3,
        INTAKE3,
        BACK_LAUNCH_1,
        END
    }

    public static final Pose startPose = new Pose(
            20.5,
            122.8,
            Math.toRadians(142.5)
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
            15,
            84,
            0
    );

    public static final Pose set2Pose = new Pose(
            48,
            60,
            Math.toRadians(180)
    );

    public static final Pose intake2Pose = new Pose(
            15,
            60,
            0
    );

    public static final Pose empty_gatePose = new Pose(
            15,
            72,
            Math.toRadians(90)
    );

    public static final Pose empty_gateControlPoint1 = new Pose(
            36,
            60,
            0
    );

    public static final Pose empty_gateControlPoint2 = new Pose(
            24,
            72,
            0
    );

    public static final Pose back_launchPose = new Pose(
            60,
            12,
            Math.toRadians(114.444)
    );

    public static final Pose set3Pose = new Pose(
            48,
            36,
            Math.toRadians(180)
    );

    public static final Pose intake3Pose = new Pose(
            15,
            36,
            0
    );

    public static final Pose endPose = new Pose(
            60,
            36,
            Math.toRadians(114.444)
    );

    public Path launchPath;
    public Path set1Path;
    public Path intake1Path;
    public Path launchPath_1;
    public Path set2Path;
    public Path intake2Path;
    public Path empty_gatePath;
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

        empty_gatePath = new Path(
                new BezierCurve(intake2Pose,
                        empty_gateControlPoint1,
                        empty_gateControlPoint2,
                        empty_gatePose)
        );
        empty_gatePath.setLinearHeadingInterpolation(
                intake2Pose.getHeading(),
                empty_gatePose.getHeading()
        );

        back_launchPath = new Path(
                new BezierCurve(empty_gatePose,
                        empty_gateControlPoint2,
                        back_launchPose)
        );
        back_launchPath.setLinearHeadingInterpolation(
                empty_gatePose.getHeading(),
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
                    setPathState(PathState.SET1);
                    break;
                }

            case SET1:
                follower.followPath(set1Path);
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
                setPathState(PathState.EMPTY_GATE);
                break;

            case EMPTY_GATE:
                follower.followPath(empty_gatePath);
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
        follower = Constants.createPedroFollower(hardwareMap);
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