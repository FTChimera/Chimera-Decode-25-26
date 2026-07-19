package org.firstinspires.ftc.teamcode.NewBot;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
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
@Autonomous(name = "NewBotAutoCloseSecond", group = "NewBotAuto", preselectTeleOp = "NewBotTeleOp")
public class NewBotAutoCloseSecond extends OpMode {
    private Follower follower;
    private LimelightSystem limelight;
    private RGBIndicator rgbIndicator;
    private AutoAlignSystem autoAlign;
    private Timer launcherTimer;

    private DcMotorEx OuttakeMotor;
    private DcMotor intakeMotor;
    private DcMotor transferMotor;

    private double deltaTime;
    private long lastTimeNs;
    private boolean shouldAutoAlign = false;

    private final Pose startPose = new Pose(123.9, 122.2, Math.toRadians(42));
    private final Pose launchPose = new Pose(94, 94, Math.toRadians(42));
    private final Pose intakePrep3 = new Pose(100.4, 37, Math.toRadians(0));
    private final Pose red3intake = new Pose(141, 37, Math.toRadians(0));
    public static final Pose finalPose = new Pose(92.3, 121.5, Math.toRadians(0));

    private Path pathOne;
    private Path pathTwo;
    private Path pathThree;
    private Path pathFour;
    private Path pathFive;

    private final double PREWARM_VELOCITY = 800;
    private double targetVelocity = 1070;
    private final double FEED_DURATION_SECONDS = 2.2;
    private final double MAX_RPM_WAIT_TIME_SECONDS = 1.2;

    @Override
    public void init() {
        Scheduler.reset();

        autoAlign = new AutoAlignSystem(Constants.AllianceColor.RED);
        limelight = new LimelightSystem(hardwareMap);
        rgbIndicator = new RGBIndicator(hardwareMap);
        autoAlign.LimelightSetUp(limelight);
        launcherTimer = new Timer();

        OuttakeMotor = hardwareMap.get(DcMotorEx.class, "OuttakeMotor");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        transferMotor = hardwareMap.dcMotor.get("transferMotor");

        OuttakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        OuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        OuttakeMotor.setZeroPowerBehavior(FLOAT);
        transferMotor.setZeroPowerBehavior(FLOAT);
        OuttakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.LaunchPIDF);

        follower = Constants.createPedroFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        lastTimeNs = System.nanoTime();
        limelight.start(0);
        OuttakeMotor.setVelocity(PREWARM_VELOCITY);
        follower.setMaxPower(Constants.AUTO_MAX_POWER);
        Scheduler.schedule(autoRoutine());
    }

    @Override
    public void loop() {
        limelight.LLUpdate();
        rgbIndicator.updateUsingLL(limelight);

        long currentTimeNs = System.nanoTime();
        deltaTime = (currentTimeNs - lastTimeNs) / 1e9;
        lastTimeNs = currentTimeNs;

        follower.update();
        Scheduler.execute();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("target velocity", targetVelocity);
        telemetry.addData("outtake velocity", OuttakeMotor.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        MatchState.setEndPose(follower.getPose());
        MatchState.setAutoTypeInfo(Constants.AllianceColor.RED, false);
    }

    private void buildPaths() {
        pathOne = new Path(new BezierLine(startPose, launchPose));
        pathOne.setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading());

        pathTwo = new Path(new BezierLine(launchPose, intakePrep3));
        pathTwo.setLinearHeadingInterpolation(launchPose.getHeading(), intakePrep3.getHeading());

        pathThree = new Path(new BezierLine(intakePrep3, red3intake));
        pathThree.setLinearHeadingInterpolation(intakePrep3.getHeading(), red3intake.getHeading());

        pathFour = new Path(new BezierLine(red3intake, launchPose));
        pathFour.setLinearHeadingInterpolation(red3intake.getHeading(), launchPose.getHeading());

        pathFive = new Path(new BezierLine(launchPose, finalPose));
        pathFive.setLinearHeadingInterpolation(launchPose.getHeading(), finalPose.getHeading());
    }

    private Command autoRoutine() {
        return sequential(
                followPath(pathOne),
                launcherSequence(),
                instant(this::IntakeStop),

                instant(this::Intake),
                followPath(pathTwo),
                followPath(pathThree, 0.7 * Constants.AUTO_MAX_POWER),
                instant(this::PREWARM),
                instant(this::IntakeStop),
                followPath(pathFour),
                launcherSequence(),
                instant(this::IntakeStop),
                followPath(pathFive),
                instant(this::IntakeStop),
                instant(this::requestOpModeStop)
        );
    }

    private Command launcherSequence() {
        return sequential(
                spinUpLauncherCommand(),
                feedLauncherCommand(),
                stopLauncherCommand()
        );
    }

    private Command spinUpLauncherCommand() {
        return Command.build()
                .setStart(() -> {
                    follower.startTeleOpDrive();
                    OuttakeMotor.setVelocity(targetVelocity);
                    autoAlign.reset();
                    shouldAutoAlign = false;
                    launcherTimer.resetTimer();
                })
                .setExecute(this::updateLauncherVelocityAndTurning)
                .setDone(() -> {
                    double currentVel = OuttakeMotor.getVelocity();
                    boolean isSpeedReached = currentVel >= targetVelocity - Constants.VELOCITY_TOLERANCE;
                    boolean isTimedOut = launcherTimer.getElapsedTimeSeconds() > MAX_RPM_WAIT_TIME_SECONDS;
                    return (isSpeedReached && !shouldAutoAlign) || isTimedOut;
                })
                .setEnd(_endCondition -> follower.setTeleOpDrive(0, 0, 0, false));
    }

    private Command feedLauncherCommand() {
        return Command.build()
                .setStart(() -> {
                    launcherTimer.resetTimer();
                    follower.setTeleOpDrive(0, 0, 0, false);
                    intakeMotor.setPower(1);
                    transferMotor.setPower(Constants.TRANSFER_UP_POSITION);
                })
                .setDone(() -> launcherTimer.getElapsedTimeSeconds() >= FEED_DURATION_SECONDS);
    }

    private Command stopLauncherCommand() {
        return instant(() -> {
            OuttakeMotor.setVelocity(Constants.STOP_VELOCITY);
            IntakeStop();
        });
    }

    private void updateLauncherVelocityAndTurning() {
        double distance = Double.NaN;
        LLResultTypes.FiducialResult fiducialResult = limelight.getResultForTag(Constants.AllianceColor.RED.getTagID());
        if (fiducialResult != null) {
            distance = limelight.calculateDistance(fiducialResult);
            if (distance == 0) {
                distance = limelight.dist;
            }
        }

        targetVelocity = 0.3 * (Double.isNaN(distance) ? targetVelocity : VelocityCalculator.NEWBOT.calculateVelocity(distance))
                + 0.7 * targetVelocity;
        OuttakeMotor.setVelocity(targetVelocity);

        double rotationCmd = -0.5 * autoAlign.getTurningPowerLimelight(deltaTime);
        if (Math.abs(limelight.tx) <= 1) {
            shouldAutoAlign = false;
        }
        if (!shouldAutoAlign) {
            rotationCmd = 0;
        }
        follower.setTeleOpDrive(0, 0, rotationCmd, false);
    }

    private Command followPath(Path path) {
        return follow(follower, new PathChain(path), Constants.AUTO_MAX_POWER);
    }

    private Command followPath(Path path, double maxPower) {
        return follow(follower, new PathChain(path), maxPower);
    }

    public void Intake() {
        intakeMotor.setPower(1);
    }

    public void IntakeStop() {
        intakeMotor.setPower(0);
        transferMotor.setPower(Constants.TRANSFER_DOWN_POSITION);
    }

    public void IntakeSafe() {
        intakeMotor.setPower(1);
        transferMotor.setPower(-0.7);
    }

    public void PREWARM() {
        OuttakeMotor.setVelocity(PREWARM_VELOCITY);
    }
}
