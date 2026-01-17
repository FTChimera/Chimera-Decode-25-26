package org.firstinspires.ftc.teamcode2.Auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode2.Systems.Consts;
import org.firstinspires.ftc.teamcode2.pedroPathing.Constants;

@Autonomous(name = "Blue Auto", group = "Pedro Auto")
public class BLUE_AUTO extends OpMode {
    DcMotor intakeMotor;
    DcMotorEx launcherMotor;

    private Follower follower;
    private Timer autonomousTimer;
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
        EMPTY_GATE,
        BACK_LAUNCH,
        SET3,
        INTAKE3,
        BACK_LAUNCH_1,
        END
    }

    private final Pose startPose = new Pose(
            15.75,
            111.27,
            Math.toRadians(180)
    );

    private final Pose launchPose = new Pose(
            36,
            108,
            Math.toRadians(135)
    );

    private final Pose set1Pose = new Pose(
            42,
            84,
            Math.toRadians(180)
    );

    private final Pose intake1Pose = new Pose(
            15,
            84,
            0
    );

    private final Pose set2Pose = new Pose(
            42,
            60,
            Math.toRadians(180)
    );

    private final Pose intake2Pose = new Pose(
            15,
            60,
            0
    );

    private final Pose empty_gatePose = new Pose(
            10,
            72,
            Math.toRadians(90)
    );

    private final Pose empty_gateControlPoint1 = new Pose(
            34,
            72,
            0
    );

    private final Pose empty_gateControlPoint2 = new Pose(
            24,
            72,
            0
    );

    private final Pose back_launchPose = new Pose(
            60,
            12,
            Math.toRadians(114.4439547804)
    );

    private final Pose set3Pose = new Pose(
            42,
            36,
            Math.toRadians(180)
    );

    private final Pose intake3Pose = new Pose(
            15,
            36,
            0
    );

    public static final Pose endPose = new Pose(
            60,
            36,
            Math.toRadians(114.4439547804)
    );

    private Path launchPath;
    private Path set1Path;
    private Path intake1Path;
    private Path launchPath_1;
    private Path set2Path;
    private Path intake2Path;
    private Path empty_gatePath;
    private Path back_launchPath;
    private Path set3Path;
    private Path intake3Path;
    private Path back_launchPath_1;
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
    }

    public void autonomousPathUpdate(boolean goToEnd) throws InterruptedException {
        if (follower.isBusy() && !goToEnd) return;

        switch (pathState) {
            case LAUNCH:
                follower.followPath(launchPath);
                runLauncherSequence(false);
                setPathState(PathState.SET1);
                break;

            case SET1:
                follower.followPath(set1Path);
                intake(true);
                setPathState(PathState.INTAKE1);
                break;

            case INTAKE1:
                follower.followPath(intake1Path);
                intake(false);
                setPathState(PathState.LAUNCH_1);
                break;

            case LAUNCH_1:
                follower.followPath(launchPath_1);
                runLauncherSequence(false);
                setPathState(PathState.SET2);
                break;

            case SET2:
                follower.followPath(set2Path);
                intake(true);
                setPathState(PathState.INTAKE2);
                break;

            case INTAKE2:
                follower.followPath(intake2Path);
                intake(false);
                setPathState(PathState.EMPTY_GATE);
                break;

            case EMPTY_GATE:
                follower.followPath(empty_gatePath);
                setPathState(PathState.BACK_LAUNCH);
                break;

            case BACK_LAUNCH:
                follower.followPath(back_launchPath);
                runLauncherSequence(true);
                setPathState(PathState.SET3);
                break;

            case SET3:
                follower.followPath(set3Path);
                intake(true);
                setPathState(PathState.INTAKE3);
                break;

            case INTAKE3:
                follower.followPath(intake3Path);
                intake(false);
                setPathState(PathState.BACK_LAUNCH_1);
                break;

            case BACK_LAUNCH_1:
                follower.followPath(back_launchPath_1);
                if (autonomousTimer.getElapsedTimeSeconds() > 25) {
                    launchOneBall(true);
                } else {
                    runLauncherSequence(true);
                }
                setPathState(PathState.END);
                break;

            case END:
                follower.followPath(
                        new Path(new BezierLine(follower.getPose(), endPose))
                );
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
        autonomousTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        //SET DIRECTION FOR MOTORS
        launcherMotor.setZeroPowerBehavior(FLOAT);
        launcherMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                Consts.LaunchPIDF
        );
        buildPaths();
        follower.setStartingPose(startPose);
        setPathState(PathState.LAUNCH);
    }
    @Override
    public void start() {
        autonomousTimer.resetTimer();
    }
    @Override
    public void loop() {
        follower.update();
        try {
            autonomousPathUpdate(false);
        } catch (InterruptedException e) {
            telemetry.addData("COULD NOT RUN PATH UPDATE", e.getMessage());
        }
        if (autonomousTimer.getElapsedTimeSeconds() > 28 && pathState != PathState.END) {
            launcherMotor.setPower(0);
            intakeMotor.setPower(0);
            setPathState(PathState.END);
            try {
                autonomousPathUpdate(true);
            } catch (InterruptedException e) {
                telemetry.addData("COULD NOT RUN PATH UPDATE", e.getMessage());
            }
        }
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void intake(boolean runIntake) {
        intakeMotor.setPower(runIntake?1:0);
    }

    public void runLauncherSequence(boolean back) throws InterruptedException {
        launcherMotor.setVelocity(back?
                Consts.MIN_VELOCITY_BACK_LAUNCH_ZONE:
                Consts.MIN_VELOCITY_FRONT_LAUNCH_ZONE);
        launcherMotor.setVelocity(back?
                Consts.TARGET_VELOCITY_BACK_LAUNCH_ZONE:
                Consts.TARGET_VELOCITY_FRONT_LAUNCH_ZONE);
        // Add push logic here for 3 balls
        // Don't forget to check for end time as this method will take time
        sleep(200);
        launcherMotor.setVelocity(Consts.STOP_VELOCITY);
    }

    public void launchOneBall(boolean back) throws InterruptedException {
        launcherMotor.setVelocity(back?
                Consts.MIN_VELOCITY_BACK_LAUNCH_ZONE:
                Consts.MIN_VELOCITY_FRONT_LAUNCH_ZONE);
        launcherMotor.setVelocity(back?
                Consts.TARGET_VELOCITY_BACK_LAUNCH_ZONE:
                Consts.TARGET_VELOCITY_FRONT_LAUNCH_ZONE);
        // Add push logic here for 1 ball
        // No need to check for end time as this method will be quick and already called within time limits
        sleep(200);
        launcherMotor.setVelocity(Consts.STOP_VELOCITY);
    }
}