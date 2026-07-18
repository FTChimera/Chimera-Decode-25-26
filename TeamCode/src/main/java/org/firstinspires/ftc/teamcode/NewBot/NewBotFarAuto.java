package org.firstinspires.ftc.teamcode.NewBot;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode.Systems.RGBIndicator;

@SuppressWarnings("SpellCheckingInspection")
@Autonomous(name = "NewBotAutoFar", group = "NewBotAuto", preselectTeleOp = "NewBotTeleOp")
public class NewBotFarAuto extends OpMode {

    private Follower follower;
    private Timer launcherTimer, waitTimer;
    // calculate dt for turning with limelight
    long lastTime;
    long currentTime;
    double deltaTime;
    boolean speedReached, timeout, firstShoot=false;
    // ---------------------------------------

    private LimelightSystem limelight;
    private RGBIndicator rgbIndicator;
    private AutoAlignSystem autoAlign; private boolean shouldAutoAlign = false;

    private DcMotorEx OuttakeMotor;
    private DcMotor intakeMotor;
    private DcMotor transferMotor;

    private Constants.AllianceColor allianceColor = Constants.AllianceColor.RED;   // default
    private boolean isLauncherRunning = false;
    private int launcherStage = 0;
    private int intake_iterations = 2;
    private boolean first_iteration = false;
    double PREWARM_VELOCITY = 1200;
    double TARGET_VELOCITY = 1375;
    final double FEED_DURATION_SECONDS = 3;
    final double WAIT_FIRST_ITERATION = 3;
    final double MAX_RPM_WAIT_TIME_SECONDS = 3;
    final double INTAKE_WAIT_TIME_SECONDS = 2;
    final double WAIT_TIME_BETWEEN_ITERATION_SECONDS = 0.5;

    // ---------------- RED FAR POSES ----------------
    private final Pose redStart = new Pose(86.591, 9.184, Math.toRadians(90));
    private final Pose redLaunch = new Pose(91.000, 12.000, Math.toRadians(70.000));
    private final Pose redPark = new Pose(104.000, 18.000, Math.toRadians(90));

    // ---------------- BLUE FAR POSES ----------------
    private final Pose blueStart = new Pose(57.409, 9.184, Math.toRadians(90));
    private final Pose blueLaunch = new Pose(56.000, 12.000, Math.toRadians(110.000));
    private final Pose bluePark = new Pose(40.000, 18.000, Math.toRadians(90));

    // ---------------- RED HUMAN INTAKE ----------------
    private final Pose redHuman = new Pose(137.550, 14.000, Math.toRadians(0));
    private final Pose redIntakePrepPrep = new Pose(116.444, 10.500, Math.toRadians(0));
    private final Pose redIntakePrep = new Pose(131.893, 12.000, Math.toRadians(350));
    private final Pose redHuman2 = new Pose(137, 16.550, Math.toRadians(0));
    private final Pose redIntakePrepPrep2 = new Pose(116.444, 12.000, Math.toRadians(0));
    private final Pose redIntakePrep2 = new Pose(131.893, 14.500, Math.toRadians(350));

    // ---------------- BLUE HUMAN INTAKE ----------------
    private final Pose blueHuman = new Pose(8.000, 13.000, Math.toRadians(180));
    private final Pose blueIntakePrepPrep = new Pose(27.556, 10.500, Math.toRadians(180));
    private final Pose blueIntakePrep = new Pose(12.107, 12.000, Math.toRadians(200));
    private final Pose blueHuman2 = new Pose(9.000, 16.550, Math.toRadians(180));
    private final Pose blueIntakePrepPrep2 = new Pose(27.556, 12.000, Math.toRadians(180));
    private final Pose blueIntakePrep2 = new Pose(12.107, 14.500, Math.toRadians(210));

    private Path toLaunch;
    private Path toPark;

    private PathChain humanIntakeChain;
    private PathChain returnToLaunchChain;
    private PathChain humanIntakeChain2;
    private PathChain returnToLaunchChain2;
    Pose startPose, launchPose, parkPose, humanPose, intakePrepPrepPose, intakePrepPose, humanPose2, intakePrepPrepPose2, intakePrepPose2;

    private int autoStage = 0; // 0 = first launch, 1 = go intake, 2 = second launch, 3 = park

    @Override
    public void init() {

        limelight = new LimelightSystem(hardwareMap);
        rgbIndicator = new RGBIndicator(hardwareMap);

        launcherTimer = new Timer();

        OuttakeMotor = hardwareMap.get(DcMotorEx.class, "OuttakeMotor");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        transferMotor = hardwareMap.dcMotor.get("transferMotor");

        OuttakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        OuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        OuttakeMotor.setZeroPowerBehavior(FLOAT);
        transferMotor.setZeroPowerBehavior(FLOAT);

        OuttakeMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                Constants.LaunchPIDF
        );

        follower = Constants.createPedroFollower(hardwareMap);

        telemetry.update();
    }

    @Override
    public void init_loop() {

        if (gamepad1.a) {
            allianceColor = Constants.AllianceColor.RED;
        }
        if (gamepad1.b) {
            allianceColor = Constants.AllianceColor.BLUE;
        }

        // Alliance based RGB
        if (allianceColor == Constants.AllianceColor.RED) {
            rgbIndicator.setColor(RGBIndicator.Color.RED);
        } else {
            rgbIndicator.setColor(RGBIndicator.Color.BLUE);
        }

        telemetry.addLine("Press A for RED");
        telemetry.addLine("Press B for BLUE");
        telemetry.addData("Alliance", allianceColor);
        telemetry.update();
    }
    public void buildPaths() {
        toLaunch = new Path(new BezierLine(startPose, launchPose));
        toLaunch.setLinearHeadingInterpolation(
                startPose.getHeading(),
                launchPose.getHeading()
        );

        toPark = new Path(new BezierLine(launchPose, parkPose));
        toPark.setLinearHeadingInterpolation(
                launchPose.getHeading(),
                parkPose.getHeading()
        );

        humanIntakeChain = follower.pathBuilder()

                // Launch -> IntakePrepPrep
                .addPath(
                        new BezierLine(
                                launchPose,
                                intakePrepPrepPose
                        )
                )
                .setLinearHeadingInterpolation(
                        launchPose.getHeading(),
                        intakePrepPrepPose.getHeading()
                )

                // IntakePrepPrep -> IntakePrep
                .addPath(
                        new BezierLine(
                                intakePrepPrepPose,
                                intakePrepPose
                        )
                )
                .setLinearHeadingInterpolation(
                        intakePrepPrepPose.getHeading(),
                        intakePrepPose.getHeading()
                )

                // IntakePrep -> Human Intake
                .addPath(
                        new BezierLine(
                                intakePrepPose,
                                humanPose
                        )
                )
                .setLinearHeadingInterpolation(
                        intakePrepPose.getHeading(),
                        humanPose.getHeading()
                )

                .build();

        returnToLaunchChain = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                humanPose,
                                intakePrepPrepPose
                        )
                )
                .setLinearHeadingInterpolation(
                        humanPose.getHeading(),
                        intakePrepPrepPose.getHeading()
                )
                .addPath(
                        new BezierLine(
                                intakePrepPrepPose,
                                launchPose
                        )
                )
                .setLinearHeadingInterpolation(
                        intakePrepPrepPose.getHeading(),
                        launchPose.getHeading()
                )
                .build();
        humanIntakeChain2 = follower.pathBuilder()

                // Launch -> IntakePrepPrep
                .addPath(
                        new BezierLine(
                                launchPose,
                                intakePrepPrepPose2
                        )
                )
                .setLinearHeadingInterpolation(
                        launchPose.getHeading(),
                        intakePrepPrepPose2.getHeading()
                )

                // IntakePrepPrep -> IntakePrep
                .addPath(
                        new BezierLine(
                                intakePrepPrepPose2,
                                intakePrepPose2
                        )
                )
                .setLinearHeadingInterpolation(
                        intakePrepPrepPose2.getHeading(),
                        intakePrepPose2.getHeading()
                )

                // IntakePrep -> Human Intake
                .addPath(
                        new BezierLine(
                                intakePrepPose2,
                                humanPose2
                        )
                )
                .setLinearHeadingInterpolation(
                        intakePrepPose2.getHeading(),
                        humanPose2.getHeading()
                )

                .build();

        returnToLaunchChain2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                humanPose2,
                                intakePrepPrepPose2
                        )
                )
                .setLinearHeadingInterpolation(
                        humanPose2.getHeading(),
                        intakePrepPrepPose2.getHeading()
                )
                .addPath(
                        new BezierLine(
                                intakePrepPrepPose2,
                                launchPose
                        )
                )
                .setLinearHeadingInterpolation(
                        intakePrepPrepPose2.getHeading(),
                        launchPose.getHeading()
                )
                .build();
    }

    @Override
    public void stop() {
        MatchState.setEndPose(follower.getPose());
        MatchState.setAutoTypeInfo(allianceColor, true);
    }

    @Override
    public void start() {

        // Initialize Auto Align System AFTER Alliance Color is set
        autoAlign = new AutoAlignSystem(allianceColor);
        autoAlign.LimelightSetUp(limelight);
        // calculate dt for turning with limelight
        lastTime = System.nanoTime();
        // ---------------------------------------

        limelight.start(0);
        boolean isRedAlliance = allianceColor == Constants.AllianceColor.RED;
        startPose = isRedAlliance ? redStart : blueStart;
        launchPose = isRedAlliance ? redLaunch : blueLaunch;
        parkPose = isRedAlliance ? redPark : bluePark;
        humanPose = isRedAlliance ? redHuman : blueHuman;
        intakePrepPrepPose = isRedAlliance ? redIntakePrepPrep : blueIntakePrepPrep;
        intakePrepPose = isRedAlliance ? redIntakePrep : blueIntakePrep;
        humanPose2 = isRedAlliance ? redHuman2 : blueHuman2;
        intakePrepPrepPose2 = isRedAlliance ? redIntakePrepPrep2 : blueIntakePrepPrep2;
        intakePrepPose2 = isRedAlliance ? redIntakePrep2 : blueIntakePrep2;

        buildPaths();

        follower.setStartingPose(startPose);
        follower.followPath(toLaunch);
        OuttakeMotor.setVelocity(TARGET_VELOCITY);
        follower.setMaxPower(Constants.AUTO_MAX_POWER);
    }

    @Override
    public void loop() {

        limelight.LLUpdate();
        rgbIndicator.updateUsingLL(limelight);
        // Calculate dt
        currentTime = System.nanoTime();
        deltaTime = (currentTime - lastTime) / 1e9; // convert to seconds
        lastTime = currentTime; // Update lastTime each loop so dt is meaningful

        switch (autoStage) {

            case 0: // launch
                if (!follower.isBusy()) {
                    if (runLauncherSequence()) {
                        if (intake_iterations > 0) {
                            waitTimer = new Timer();
                            autoStage = 1;
                        } else {
                            follower.followPath(toPark);
                            autoStage = 4;
                        }
                    }
                }
                break;
            case 1:
                if (waitTimer.getElapsedTimeSeconds() >= WAIT_TIME_BETWEEN_ITERATION_SECONDS || !first_iteration) {
                    follower.followPath(!first_iteration ? humanIntakeChain : humanIntakeChain2);
                    intakeMotor.setPower(1);
                    transferMotor.setPower(-0.8); // safe intake
                    waitTimer = null;
                    autoStage = 2;
                }
                break;
            case 2:
                if (!follower.isBusy() && waitTimer == null) {
                    waitTimer = new Timer();
                }
                if (!follower.isBusy() && waitTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_TIME_SECONDS) {
                    autoStage = 3;
                    waitTimer = null;
                }
                break;
            case 3: // Intake 3 from human
                if (!follower.isBusy()) {
                    follower.followPath(!first_iteration ? returnToLaunchChain : returnToLaunchChain2);
                    intake_iterations --;
                    first_iteration = true; // done with first iteration if we just did it
                    autoStage = 0; // launch
                }
                break;

            case 4: // Parking
                if (!follower.isBusy()) {
                    // Done with auto so lets stop it.
                    requestOpModeStop();
                }
                break;
        }

        follower.update();

        telemetry.addData("Alliance", allianceColor);
        telemetry.addData("Auto Stage", autoStage);
        telemetry.addData("Iteration number (counting down)", intake_iterations);
        telemetry.addData("Velocity", OuttakeMotor.getVelocity());
        telemetry.update();
    }


    // ------------------------------------------------
    // Launcher Logic
    // ------------------------------------------------

    public boolean runLauncherSequence() {
        if (!isLauncherRunning) {
            //TARGET_VELOCITY = VelocityCalculator.NEWBOT.calculateVelocity(limelight.dist);
            OuttakeMotor.setVelocity(TARGET_VELOCITY);
            // Auto Align for Far Auto Start
            autoAlign.reset();
            shouldAutoAlign = true;
            follower.startTeleopDrive(); // explicity switch to teleop mode
            isLauncherRunning = true;
            launcherStage = 0;
            launcherTimer.resetTimer();
            return false;
        }

        switch (launcherStage) {

            case 0:
                handleVelocityAndAutoAlignLoop();
                 //If speed reached OR timed out, start feeding
                if ((speedReached&&!shouldAutoAlign || timeout) && firstShoot || (!firstShoot &&launcherTimer.getElapsedTimeSeconds() >= WAIT_FIRST_ITERATION)) {
                    launcherStage = 1;
                    firstShoot = true;
                    launcherTimer.resetTimer();
                }
                break;

            case 1:
                handleVelocityAndAutoAlignLoop();
                intakeMotor.setPower(1);
                transferMotor.setPower(Constants.TRANSFER_UP_POSITION);

                if (launcherTimer.getElapsedTimeSeconds() >= FEED_DURATION_SECONDS) {
                    launcherStage = 2;
                }
                break;

            case 2:
                OuttakeMotor.setVelocity(PREWARM_VELOCITY);
                intakeMotor.setPower(0);
                transferMotor.setPower(Constants.TRANSFER_DOWN_POSITION);

                isLauncherRunning = false;
                return true;
        }

        return false;
    }

    public void handleVelocityAndAutoAlignLoop()
    {
        // calculate dynamic velocity
        // apply lowpass filter
        // Distance calculation
        double distance = Double.NaN;
        LLResultTypes.FiducialResult fiducialResult = limelight.getResultForTag(allianceColor.getTagID());
        if (!(fiducialResult==null)) {
            distance = limelight.calculateDistance(fiducialResult);
            if (distance == 0) {
                distance = limelight.dist; // SAFETY CHECK
            }
        }
        TARGET_VELOCITY = 0.3*(Double.isNaN(distance) ? TARGET_VELOCITY : VelocityCalculator.NEWBOT.calculateVelocity(distance))  + 0.7*TARGET_VELOCITY;
        double currentVel = OuttakeMotor.getVelocity();
        double threshold = TARGET_VELOCITY - Constants.VELOCITY_TOLERANCE;

        speedReached = currentVel >= threshold;
        timeout = launcherTimer.getElapsedTimeSeconds() > MAX_RPM_WAIT_TIME_SECONDS;

        // Auto Align Logic
        double rotationCmd = -0.8*autoAlign.getTurningPowerLimelight(deltaTime)/follower.getMaxPowerScaling();
        if (Math.abs(limelight.tx) <= 1) shouldAutoAlign = false;
        if (!shouldAutoAlign) rotationCmd = 0;
        follower.setTeleOpDrive(0,0, rotationCmd, false); // keep calling method
    }
}
