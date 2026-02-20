package org.firstinspires.ftc.teamcode.NewBot;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode.Systems.RGBIndicator;

@Autonomous(name = "NewBotAutoFar", group = "NewBotAuto")
public class NewBotFarAuto extends OpMode {

    private Follower follower;
    private Timer launcherTimer;

    private LimelightSystem limelight;
    private RGBIndicator rgbIndicator;

    private DcMotorEx OuttakeMotor;
    private DcMotor intakeMotor;
    private DcMotor transferMotor;

    private Constants.AllianceColor allianceColor = Constants.AllianceColor.RED;   // default
    private boolean isLauncherRunning = false;
    private int launcherStage = 0;

    double TARGET_VELOCITY = 1200;
    final double FEED_DURATION_SECONDS = 2;
    final double MAX_RPM_WAIT_TIME_SECONDS = 1.2;

    // ---------------- RED FAR POSES ----------------
    private final Pose redStart = new Pose(86.591, 9.184, Math.toRadians(90));
    private final Pose redLaunch = new Pose(84.000, 12.000, Math.toRadians(65.5560452));
    private final Pose redPark = new Pose(104.000, 12.000, Math.toRadians(90));

    // ---------------- BLUE FAR POSES ----------------
    private final Pose blueStart = new Pose(57.409, 9.184, Math.toRadians(90));
    private final Pose blueLaunch = new Pose(60.000, 12.000, Math.toRadians(114.4439548));
    private final Pose bluePark = new Pose(40.000, 12.000, Math.toRadians(90));

    // ---------------- RED HUMAN INTAKE ----------------
    private final Pose redHuman = new Pose(134.360, 9.428, Math.toRadians(0));
    private final Pose redIntakePrepPrep = new Pose(116.444, 13.336, Math.toRadians(0));
    private final Pose redIntakePrep = new Pose(131.893, 11.366, Math.toRadians(350));

    // ---------------- BLUE HUMAN INTAKE ----------------
    private final Pose blueHuman = new Pose(9.640, 9.428, Math.toRadians(180));
    private final Pose blueIntakePrepPrep = new Pose(27.556, 13.336, Math.toRadians(180));
    private final Pose blueIntakePrep = new Pose(12.107, 11.366, Math.toRadians(190));

    private Path toLaunch;
    private Path toPark;

    private PathChain humanIntakeChain;
    private PathChain returnToLaunchChain;

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

        telemetry.addLine("Press A for RED");
        telemetry.addLine("Press B for BLUE");
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

        telemetry.addData("Alliance", allianceColor);
        telemetry.update();
    }
    @Override
    public void start() {

        limelight.start(0);
        boolean isRedAlliance = allianceColor == Constants.AllianceColor.RED;
        Pose startPose = isRedAlliance ? redStart : blueStart;
        Pose launchPose = isRedAlliance ? redLaunch : blueLaunch;
        Pose parkPose = isRedAlliance ? redPark : bluePark;
        Pose humanPose = isRedAlliance ? redHuman : blueHuman;
        Pose intakePrepPrepPose = isRedAlliance ? redIntakePrepPrep : blueIntakePrepPrep;
        Pose intakePrepPose = isRedAlliance ? redIntakePrep : blueIntakePrep;

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
                                launchPose
                        )
                )
                .setLinearHeadingInterpolation(
                        humanPose.getHeading(),
                        launchPose.getHeading()
                )
                .build();

        follower.setStartingPose(startPose);
        follower.followPath(toLaunch);
    }

    @Override
    public void loop() {

        limelight.LLUpdate();
        rgbIndicator.updateUsingLL(limelight);
        follower.update();

        switch (autoStage) {

            case 0: // First launch
                if (!follower.isBusy()) {
                    if (runLauncherSequence()) {
                        follower.followPath(humanIntakeChain);
                        intakeMotor.setPower(1);
                        autoStage = 1;
                    }
                }
                break;

            case 1: // Intake 3 from human
                if (!follower.isBusy()) {
                    follower.followPath(returnToLaunchChain);
                    autoStage = 2;
                }
                break;

            case 2: // Second launch
                if (!follower.isBusy()) {
                    if (runLauncherSequence()) {
                        intakeMotor.setPower(0);
                        follower.followPath(toPark);
                        autoStage = 3;
                    }
                }
                break;

            case 3: // Parking
                if (!follower.isBusy()) {
                    // Done with auto so lets stop it.
                    requestOpModeStop();
                }
                break;
        }

        telemetry.addData("Alliance", allianceColor);
        telemetry.addData("Auto Stage", autoStage);
        telemetry.addData("Velocity", OuttakeMotor.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {}

    // ------------------------------------------------
    // Launcher Logic (copied + adapted)
    // ------------------------------------------------

    public boolean runLauncherSequence() {

        if (!isLauncherRunning) {
            //TARGET_VELOCITY = VelocityCalculator.NEWBOT.calculateVelocity(limelight.dist);
            OuttakeMotor.setVelocity(TARGET_VELOCITY);
            isLauncherRunning = true;
            launcherStage = 0;
            launcherTimer.resetTimer();
            return false;
        }

        switch (launcherStage) {

            case 0:
                double currentVel = OuttakeMotor.getVelocity();
                double threshold = TARGET_VELOCITY - Constants.VELOCITY_TOLERANCE;

                boolean speedReached = currentVel >= threshold;
                boolean timeout = launcherTimer.getElapsedTimeSeconds() > MAX_RPM_WAIT_TIME_SECONDS;

                if (speedReached || timeout) {
                    launcherStage = 1;
                    launcherTimer.resetTimer();
                }
                break;

            case 1:
                intakeMotor.setPower(1);
                transferMotor.setPower(Constants.TRANSFER_UP_POSITION);

                if (launcherTimer.getElapsedTimeSeconds() >= FEED_DURATION_SECONDS) {
                    launcherStage = 2;
                }
                break;

            case 2:
                OuttakeMotor.setVelocity(Constants.STOP_VELOCITY);
                intakeMotor.setPower(0);
                transferMotor.setPower(Constants.TRANSFER_DOWN_POSITION);

                isLauncherRunning = false;
                return true;
        }

        return false;
    }
}