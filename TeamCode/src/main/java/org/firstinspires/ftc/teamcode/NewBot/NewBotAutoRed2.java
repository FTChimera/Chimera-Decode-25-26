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

import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode.Systems.RGBIndicator;

@SuppressWarnings("SpellCheckingInspection")
@Autonomous(name = "NewBotAutoRed", group = "NewBotAuto", preselectTeleOp = "NewBotTeleOp")
public class NewBotAutoRed2 extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer, launcherTimer;
    private int pathState, launcherStage = 0, gate_iterations = 2;
    private LimelightSystem limelight; private RGBIndicator rgbIndicator;

    // Poses remain unchanged from your original file
    private final Pose startPose = new Pose(123.9, 122.2, Math.toRadians(42));
    private final Pose launchPose = new Pose(99, 99, Math.toRadians(42));
    private final Pose intakePrep = new Pose(96.4, 87, Math.toRadians(0));
    private final Pose red1Intake = new Pose(127, 87, Math.toRadians(0));
    private final Pose intakePrep2 = new Pose(101.0, 61, Math.toRadians(0));
    private final Pose red2Intake = new Pose(138, 61, Math.toRadians(0));
    private final Pose launchControl = new Pose(108.4, 41.4, Math.toRadians(0));
    private final Pose intakePrep3 = new Pose(100.4, 37, Math.toRadians(0));
    private final Pose red3intake = new Pose(136, 37, Math.toRadians(0));
    private final Pose launchControl1 = new Pose(108.5,36.8, Math.toRadians(0));
    // --- NEW GATE POSES (assumed positions; adjust if required) ---
    private final Pose gateOpener = new Pose(129.5, 70, Math.toRadians(0));
    private final Pose intakeGate = new Pose(128, 60, Math.toRadians(55));
    private final Pose gatelaunchControl = new Pose(114.3, 70, Math.toRadians(0));
    private final Pose gateLaunchControl2 = new Pose(116.1, 43.3, Math.toRadians(0));

    public static final Pose finalPose = new Pose(92.3, 121.5, Math.toRadians(0));

    private Path pathOne, pathTwo, pathThree, pathFour, pathFive, pathSix, pathSeven, pathEight, pathNine, pathTen, pathEleven;
    // gate paths
    private Path pathGateOne, pathGateTwo, pathGateReturn;

    // Updated Constants based on TeleOp
    // TeleOp "Front Launch" is 1100, Back is 500. Assuming Auto shoots from Front/Close range.
    double TARGET_VELOCITY = 1000;
    // Duration to run the transfer motor to ensure all balls are fired
    final double GATE_WAITING_TIME = 1000; // 1 second for intaking from gate
    final double FEED_DURATION_SECONDS = 2;
    final double MAX_RPM_WAIT_TIME_SECONDS = 1.2; // Fail-safe if RPM isn't reached

    // Path State Constants
    final int CHIMERA_LAUNCH = 1;
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
    // gate related states
    final int CHIMERA_GATE_ONE = 14;
    final int CHIMERA_GATE_TWO = 15;
    final int CHIMERA_GATE_INTAKE = 16;
    final int CHIMERA_GATE_RETURN = 17;
    final int CHIMERA_AFTER_GATE = 18;

    // post-launch routing: where to go after we finish CHIMERA_LAUNCH
    private int postLaunchState = CHIMERA_PATH_FIVE;

    // flag to detect the very first launch after starting from startPose
    private boolean initialLaunchDone = false;

    int iteration;
    boolean isLauncherRunning = false;
    // gate/runtime flags
    int gateCount = 0;
    boolean gatesDone = false;
    boolean red1Done = false;
    boolean red3Done = false;

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
        pathFive.setLinearHeadingInterpolation(launchPose.getHeading(), intakePrep2.getHeading());

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

        // Gate paths: launchPose -> gateOpener -> intakeGate -> return to launchPose via launchControl
        pathGateOne = new Path(new BezierCurve(launchPose, gatelaunchControl, gateOpener));
        pathGateOne.setLinearHeadingInterpolation(launchPose.getHeading(), gateOpener.getHeading());

        pathGateTwo = new Path(new BezierCurve(gateOpener, gateLaunchControl2, intakeGate));
        pathGateTwo.setLinearHeadingInterpolation(gateOpener.getHeading(), intakeGate.getHeading());

        pathGateReturn = new Path(new BezierCurve(intakeGate, launchControl, launchPose));
        pathGateReturn.setLinearHeadingInterpolation(intakeGate.getHeading(), launchPose.getHeading());
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
                        IntakeStop();
                        setPathState(postLaunchState);
                    }
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
                    red1Done = true;
                    setPathState(CHIMERA_PATH_EIGHT);
                    // After returning to launchPose we want to shoot first
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
                    // After returning to launchPose via launchControl, shoot then start gate chain
                    postLaunchState = CHIMERA_GATE_ONE;
                    setPathState(CHIMERA_LAUNCH);
                }
                break;
            case CHIMERA_GATE_ONE:
                if (!follower.isBusy()) {
                    follower.followPath(pathGateOne);
                    setPathState(CHIMERA_GATE_TWO);
                }
                break;
            case CHIMERA_GATE_TWO:
                if (!follower.isBusy()) {
                    follower.followPath(pathGateTwo);
                    IntakeSafe();
                    setPathState(CHIMERA_GATE_INTAKE);
                }
                break;
            case CHIMERA_GATE_INTAKE:
                if (pathTimer.getElapsedTime() >= GATE_WAITING_TIME) {
                    setPathState(CHIMERA_GATE_RETURN);
                }
                break;
            case CHIMERA_GATE_RETURN:
                if (!follower.isBusy()) {
                    follower.followPath(pathGateReturn);
                    // After returning to launchPose via gateReturn, shoot then run CHIMERA_AFTER_GATE
                    postLaunchState = CHIMERA_AFTER_GATE;
                    setPathState(CHIMERA_LAUNCH);
                }
                break;
            case CHIMERA_AFTER_GATE:
                if (!follower.isBusy()) {
                    gateCount++;
                    if (gateCount < gate_iterations) {
                        // Repeat the intake->red2->return->gate loop
                        setPathState(CHIMERA_PATH_FIVE);
                    } else {
                        // Gates finished: proceed to intake red1, then red3
                        gatesDone = true;
                        setPathState(CHIMERA_PATH_TWO);
                    }
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
                    // After returning to launchPose via red3 path, shoot then stop
                    postLaunchState = CHIMERA_STOP;
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
                if (!(iteration == 0)) IntakeSafe(); // dont intake for first iteration
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