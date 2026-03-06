package org.firstinspires.ftc.teamcode.NewBot;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode.Systems.RGBIndicator;

@SuppressWarnings("SpellCheckingInspection")
@Autonomous(name = "NewBotAutoRed", group = "NewBotAuto", preselectTeleOp = "NewBotTeleOp")
public class NewBotAutoRed extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer, launcherTimer;
    private int pathState, launcherStage = 0;
    private LimelightSystem limelight; private RGBIndicator rgbIndicator;

    // Poses remain unchanged from your original file
    private final Pose startPose = new Pose(123.9, 122.2, Math.toRadians(42));
    private final Pose launchPose = new Pose(99, 99, Math.toRadians(42));
    private final Pose intakePrep = new Pose(96.4, 87, Math.toRadians(0));
    private final Pose red1Intake = new Pose(128.5, 87, Math.toRadians(0));
    private final Pose intakePrep2 = new Pose(101.0, 61, Math.toRadians(0));
    private final Pose red2Intake = new Pose(138.5, 61, Math.toRadians(0));
    private final Pose launchControl = new Pose(108.4, 41.4, Math.toRadians(0));
    private final Pose intakePrep3 = new Pose(100.4, 37, Math.toRadians(0));
    private final Pose red3intake = new Pose(141, 37, Math.toRadians(0));
    private final Pose launchControl1 = new Pose(108.5,36.8, Math.toRadians(0));
    public static final Pose finalPose = new Pose(92.3, 121.5, Math.toRadians(0));

    private Path pathOne, pathTwo, pathThree, pathFour, pathFive, pathSix, pathSeven, pathEight, pathNine, pathTen, pathEleven;

    // Updated Constants based on TeleOp
    // TeleOp "Front Launch" is 1100, Back is 500. Assuming Auto shoots from Front/Close range.
    double TARGET_VELOCITY = 1040;
    // Duration to run the transfer motor to ensure all balls are fired
    final double FEED_DURATION_SECONDS = 2;
    final double MAX_RPM_WAIT_TIME_SECONDS = 1.2; // Fail-safe if RPM isn't reached

    // Path State Constants
    final int CHIMERA_LAUNCH = 1;
    final int CHIMERA_LAUNCH_INTAKE = 2;
    final int CHIMERA_PATH_TWO = 3;
    final int CHIMERA_PATH_THREE = 4;
    final int CHIMERA_PATH_FOUR = 5;
    final int CHIMERA_PATH_FIVE = 6;
    final int CHIMERA_PATH_SIX = 7;
    final int CHIMERA_PATH_SEVEN = 8;
    final int CHIMERA_PATH_EIGHT = 9;
    final int CHIMERA_PATH_NINE = 10;
    final int CHIMERA_PATH_TEN = 11;
    final int CHIMERA_STOP = 12;
    final int STOP_AUTO = 13;

    boolean first_iteration = false;
    boolean second_iteration = false;
    boolean third_iteration = false;
    boolean isLauncherRunning = false;

    // PID Values from TeleOp (Kp=300, Kf=10)
    final double Kp = 300;
    final double Ki = 0.0;
    final double Kd = 0.0;
    final double Kf = 10;

    // Updated Motor Declarations
    DcMotorEx OuttakeMotor;
    DcMotor intakeMotor;
    DcMotor transferMotor; // Replaces pushServo

    public void buildPaths() {
        pathOne = new Path(new BezierLine(startPose, launchPose));
        pathOne.setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading());

        pathTwo = new Path(new BezierLine(launchPose, intakePrep));
        pathTwo.setLinearHeadingInterpolation(launchPose.getHeading(), intakePrep.getHeading());

        pathThree = new Path(new BezierLine(intakePrep, red1Intake));
        pathThree.setLinearHeadingInterpolation(intakePrep.getHeading(), red1Intake.getHeading());

        pathFour = new Path(new BezierLine(red1Intake, launchPose));
        pathFour.setLinearHeadingInterpolation(red1Intake.getHeading(), launchPose.getHeading());

        pathFive = new Path(new BezierLine(launchPose, intakePrep2));
        pathFive.setLinearHeadingInterpolation(launchPose.getHeading(), red2Intake.getHeading());

        pathSix = new Path(new BezierLine(intakePrep2, red2Intake));
        pathSix.setLinearHeadingInterpolation(intakePrep2.getHeading(), red2Intake.getHeading());

        pathSeven = new Path(new BezierCurve(red2Intake, launchControl, launchPose));
        pathSeven.setLinearHeadingInterpolation(red2Intake.getHeading(), launchPose.getHeading());

        pathEight = new Path(new BezierLine(launchPose, intakePrep3));
        pathEight.setLinearHeadingInterpolation(launchPose.getHeading(), intakePrep3.getHeading());

        pathNine = new Path(new BezierLine(intakePrep3, red3intake));
        pathNine.setLinearHeadingInterpolation(intakePrep3.getHeading(), red3intake.getHeading());

        pathTen = new Path(new BezierCurve(red3intake,launchControl1, launchPose));
        pathTen.setLinearHeadingInterpolation(red3intake.getHeading(), launchPose.getHeading());

        pathEleven = new Path(new BezierLine(launchPose, finalPose));
        pathEleven.setLinearHeadingInterpolation(launchPose.getHeading(), finalPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(pathOne);
                setPathState(CHIMERA_LAUNCH);
                break;
            case CHIMERA_LAUNCH:
                if (!follower.isBusy()) {
                    // This function now handles spinning up AND feeding
                    if (runLauncherSequence()) {
                        setPathState(CHIMERA_LAUNCH_INTAKE);
                    }
                }
                break;
            case CHIMERA_LAUNCH_INTAKE:
                IntakeStop();
                if (!first_iteration) {
                    setPathState(CHIMERA_PATH_TWO);
                    first_iteration = true;
                } else if (!second_iteration) {
                    setPathState(CHIMERA_PATH_FIVE);
                    second_iteration = true;
                } else if (!third_iteration) {
                    setPathState(CHIMERA_PATH_EIGHT);
                    third_iteration = true;
                } else {
                    follower.followPath(pathEleven);
                    setPathState(CHIMERA_STOP);
                }
                break;
            case CHIMERA_PATH_TWO:
                if (!follower.isBusy()) {
                    Intake();
                    follower.followPath(pathTwo);
                    setPathState(CHIMERA_PATH_THREE);
                }
                break;
            case CHIMERA_PATH_THREE:
                if (!follower.isBusy()) {
                    follower.followPath(pathThree);
                    setPathState(CHIMERA_PATH_FOUR);
                }
                break;
            case CHIMERA_PATH_FOUR:
                if (!follower.isBusy()) {
                    follower.followPath(pathFour);
                    setPathState(CHIMERA_LAUNCH);
                    IntakeSafe();
                }
                break;
            case CHIMERA_PATH_FIVE:
                if (!follower.isBusy()) {
                    Intake();
                    follower.followPath(pathFive);
                    setPathState(CHIMERA_PATH_SIX);
                }
                break;
            case CHIMERA_PATH_SIX:
                if (!follower.isBusy()) {
                    follower.followPath(pathSix);
                    IntakeSafe();
                    setPathState(CHIMERA_PATH_SEVEN);
                }
                break;
            case CHIMERA_PATH_SEVEN:
                if (!follower.isBusy()) {
                    follower.followPath(pathSeven);
                    setPathState(CHIMERA_LAUNCH);
                }
                break;
            case CHIMERA_PATH_EIGHT:
                if (!follower.isBusy()) {
                    Intake();
                    follower.followPath(pathEight);
                    setPathState(CHIMERA_PATH_NINE);
                }
                break;
            case CHIMERA_PATH_NINE:
                if (!follower.isBusy()) {
                    follower.followPath(pathNine);
                    setPathState(CHIMERA_PATH_TEN);
                }
                break;
            case CHIMERA_PATH_TEN:
                if (!follower.isBusy()) {
                    follower.followPath(pathTen);
                    setPathState(CHIMERA_LAUNCH);
                }
                break;
            case CHIMERA_STOP:
                IntakeStop();
                if (!follower.isBusy()) {
                    setPathState(STOP_AUTO);
                }
                break;
            case STOP_AUTO:
                telemetry.addLine("Autonomous Complete");
                requestOpModeStop();
                break;
            default:
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        limelight.LLUpdate();
        rgbIndicator.updateUsingLL(limelight);
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        limelight = new LimelightSystem(hardwareMap);
        rgbIndicator = new RGBIndicator(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        launcherTimer = new Timer();

        opmodeTimer.resetTimer();

        // --- NEW MOTOR CONFIGURATION ---
        OuttakeMotor = hardwareMap.get(DcMotorEx.class, "OuttakeMotor");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        transferMotor = hardwareMap.dcMotor.get("transferMotor");

        // Directions matched to TeleOp
        OuttakeMotor.setDirection(DcMotorSimple.Direction.FORWARD); // Default in TeleOp
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Encoder Settings
        OuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Zero Power Behavior
        OuttakeMotor.setZeroPowerBehavior(FLOAT);
        transferMotor.setZeroPowerBehavior(FLOAT);

        // PID Tuning (Values from TeleOp)
        OuttakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.LaunchPIDF);

        follower = Constants.createPedroFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        limelight.start(0);
        setPathState(0);
    }

    @Override
    public void stop() {}

    /**
     * Executes the launch sequence for the NEW mechanism:
     * 1. Spin up OuttakeMotor (Flywheel).
     * 2. Wait for target velocity.
     * 3. Run TransferMotor (Feeder) for set duration.
     * 4. Stop everything.
     */
    public boolean runLauncherSequence() {
        // 1. Initialization: Start Flywheel
        if (!isLauncherRunning) {
            //TARGET_VELOCITY = VelocityCalculator.NEWBOT.calculateVelocity(limelight.dist);
            OuttakeMotor.setVelocity(TARGET_VELOCITY);
            isLauncherRunning = true;
            launcherStage = 0;
            launcherTimer.resetTimer();
            return false;
        }

        telemetry.addData("Outtake Velocity", OuttakeMotor.getVelocity());
        telemetry.addData("Target Velocity", TARGET_VELOCITY);

        // 2. State Machine
        switch (launcherStage) {
            case 0: // WAIT FOR RPM
                // FEED BALL WHILE WAITING
                if (first_iteration) IntakeSafe(); // dont intake for first iteration
                double currentVel = OuttakeMotor.getVelocity();
                double targetThreshold = TARGET_VELOCITY - Constants.VELOCITY_TOLERANCE;

                boolean isSpeedReached = (currentVel >= targetThreshold);
                boolean isTimedOut = (launcherTimer.getElapsedTimeSeconds() > MAX_RPM_WAIT_TIME_SECONDS);

                // If speed reached OR timed out, start feeding
                if (isSpeedReached || isTimedOut) {
                    launcherStage = 1;
                    launcherTimer.resetTimer();
                }
                break;

            case 1: // FEEDING (RUN TRANSFER)
                // TeleOp logic: Intake runs at 1, Transfer at 0.5
                intakeMotor.setPower(1);
                transferMotor.setPower(Constants.TRANSFER_UP_POSITION);

                // Run for defined duration to empty hopper
                if (launcherTimer.getElapsedTimeSeconds() >= FEED_DURATION_SECONDS) {
                    launcherStage = 2;
                }
                break;

            case 2: // STOP & FINISH
                OuttakeMotor.setVelocity(Constants.STOP_VELOCITY);
                intakeMotor.setPower(0);
                transferMotor.setPower(Constants.TRANSFER_DOWN_POSITION);

                isLauncherRunning = false;
                return true; // Sequence Complete
        }

        return false; // Still running
    }

    public void Intake() {
        intakeMotor.setPower(1);
    }

    public void IntakeStop() {
        intakeMotor.setPower(0);
    }
    public void IntakeSafe() {
        intakeMotor.setPower(1);
        transferMotor.setPower(-0.7); // to avoid feeding balls to launcher
    }
}