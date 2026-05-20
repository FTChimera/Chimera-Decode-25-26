package org.firstinspires.ftc.teamcode.biobuzzTest;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NewBot.AutoAlignSystem;
import org.firstinspires.ftc.teamcode.NewBot.Constants;
import org.firstinspires.ftc.teamcode.NewBot.LPF_Corrector;
import org.firstinspires.ftc.teamcode.NewBot.MatchState;
import org.firstinspires.ftc.teamcode.NewBot.PedroDrive;
import org.firstinspires.ftc.teamcode.NewBot.VelocityCalculator;
import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode.Systems.RGBIndicator;

import java.util.List;

@SuppressWarnings({"FieldCanBeLocal", "SpellCheckingInspection"})
public abstract class NewBotTeleAbstract extends LinearOpMode {
    boolean testingMode = false,
            shouldUseLimelightAutoAlign = true,
            slowMode = false;
    // declaring our PIDF tuning values
    double setTargetVelocity = 0;
    double setMinVelocity = 0;
    private PedroDrive follower;
    private LPF_Corrector poseCorrector; // Use a lowpass filter instead
    Constants.AllianceColor allianceColor;
    LimelightSystem limelight;
    AutoAlignSystem autoAlignSystem; boolean shouldAutoAlign = false;
    RGBIndicator rgbIndicator;
    int launcherStage = 0; ElapsedTime launcherTimer;
    boolean fieldCentric;

    public NewBotTeleAbstract(Constants.AllianceColor col, boolean field) {
        allianceColor = col;
        fieldCentric = field;
    }
    abstract Pose getStartingPose();
    @Override
    public void runOpMode() throws InterruptedException {
        launcherTimer = new ElapsedTime();
        // BULK READING
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        poseCorrector = new LPF_Corrector();
        limelight = new LimelightSystem(hardwareMap);
        rgbIndicator = new RGBIndicator(hardwareMap);
        rgbIndicator.setColor(RGBIndicator.Color.VIOLET);
        while (opModeInInit())
        {
            // This method is called repeatedly during the init phase
            telemetry.addLine("Press X to switch alliance, B to switch between testing mode and comp mode");
            telemetry.addData("Alliance selected: ", allianceColor);
            telemetry.addData("Is Testing Mode", testingMode);
            if (gamepad1.bWasPressed()) {
                testingMode = !testingMode;
            }
            if (gamepad1.xWasReleased()) {
                allianceColor = allianceColor.switchColors();
            }

            // color
            RGBIndicator.Color col;
            if (testingMode || MatchState.getAllianceColor() == null || MatchState.getStartingPose() == null) col = allianceColor == Constants.AllianceColor.RED ? RGBIndicator.Color.RED : RGBIndicator.Color.BLUE;
            else col = MatchState.getAllianceColor() == Constants.AllianceColor.RED ? RGBIndicator.Color.RED : RGBIndicator.Color.BLUE;
            rgbIndicator.setColor(col);
            telemetry.update();
        }



        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get(Constants.pedroMecanumDriveConstants.leftFrontMotorName);
        DcMotor backLeftMotor = hardwareMap.dcMotor.get(Constants.pedroMecanumDriveConstants.leftRearMotorName);
        DcMotor frontRightMotor = hardwareMap.dcMotor.get(Constants.pedroMecanumDriveConstants.rightFrontMotorName);
        DcMotor backRightMotor = hardwareMap.dcMotor.get(Constants.pedroMecanumDriveConstants.rightRearMotorName);
        // Using DcMotorEx instead of DcMotor to use PID controller
        DcMotorEx OuttakeMotor = hardwareMap.get(DcMotorEx.class,"OuttakeMotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor transferMotor = hardwareMap.dcMotor.get("transferMotor");

        // set up
        if (!testingMode) {
            //allianceColor = MatchState.getAllianceColor() == null ? allianceColor : MatchState.getAllianceColor();
            follower = new PedroDrive(hardwareMap, getStartingPose());
            poseCorrector.resetToPose(getStartingPose());
        } else {

            follower = new PedroDrive(hardwareMap, getStartingPose());
            poseCorrector.resetToPose(getStartingPose());
        }

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        OuttakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        OuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        OuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        OuttakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.LaunchPIDF);

        autoAlignSystem = new AutoAlignSystem(allianceColor);

        autoAlignSystem.LimelightSetUp(limelight);

        telemetry.addData("Status", "Initialized");telemetry.update();
        waitForStart();

        //if (isStopRequested()) return;
        limelight.start();
        telemetry.addData("Status", "Running");telemetry.update();

        // calculate dt for turning with limelight
        long lastTime = System.nanoTime();
        long currentTime;
        double deltaTime;
        // ---------------------------------------
        boolean shouldKeepLauncherActive = false;
        double distance = 0;
        follower.startTeleopDrive();
        while (opModeIsActive()) {
            follower.update();
            limelight.LLUpdate();
            rgbIndicator.updateUsingLL(limelight);
            //poseCorrector.updateFromLimelight(limelight, allianceColor, follower);
            if (poseCorrector.isMeasured) follower.correctPose(poseCorrector.getEstimatedPose());
            // Clear cached data from control hubs for fresh readings
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            // Pose Corrector
            if (autoAlignSystem.LLCanSeeGoal()) {
                follower.correctPose(poseCorrector.getEstimatedPose());
            }
            // Gamepad X
            if (gamepad1.xWasPressed()) {
                allianceColor = allianceColor.switchColors();
            }
            // Distance calculation
            LLResultTypes.FiducialResult fiducialResult = limelight.getResultForTag(allianceColor.getTagID());
            if (!(fiducialResult==null)) {
                distance = limelight.calculateDistance(fiducialResult);
                if (distance == 0) {
                    distance = limelight.dist; // SAFETY CHECK
                }
            }

            double y, x, rx;

            y = -gamepad1.left_stick_y * 1.01; // Remember, Y stick value is reversed
            x = -gamepad1.left_stick_x * 1.01; // Counteract imperfect strafing
            rx = -gamepad1.right_stick_x * 1.01;

            x = x * 1.1;

            double multiplier = slowMode ? 0.5 : 1.0;
            y *= Math.abs(y)*multiplier;
            x *= Math.abs(x)*multiplier;
            rx *= Math.abs(rx * multiplier*0.65);

            // Calculate dt
            currentTime = System.nanoTime();
            deltaTime = (currentTime - lastTime) / 1e9; // convert to seconds
            lastTime = currentTime; // Update lastTime each loop so dt is meaningful
            if ((gamepad1.back || gamepad2.back)) {
                shouldAutoAlign = true;
            }
            if (gamepad1.backWasReleased() || gamepad2.backWasReleased()) {
                shouldAutoAlign = false;
            }
            if (shouldAutoAlign) {
                rx = 0;
                if (!(rgbIndicator.getPWM() == RGBIndicator.GREEN_PWM)&&shouldUseLimelightAutoAlign || !shouldUseLimelightAutoAlign&&autoAlignSystem.isErrorAtTolerance()) {
                    if (shouldUseLimelightAutoAlign) rx = -autoAlignSystem.getTurningPowerLimelight(deltaTime);
                    else rx = -autoAlignSystem.getTurningPowerPose(follower.getPose(), follower.getVelocity(), deltaTime, true);
                } else {
                    shouldAutoAlign = false; // automatic stop
                }
            }

            follower.setTeleopDrive(y*0.9, x*0.9, rx*0.9, false,0);

            if (gamepad1.y || gamepad2.y) {
                handleTeleOpShootingUpdate(
                        OuttakeMotor,intakeMotor,transferMotor,
                        distance,true
                );
            } else if (gamepad1.a || gamepad2.a) {
                handleTeleOpShootingUpdate(
                        OuttakeMotor,intakeMotor,transferMotor,
                        distance,false
                );
            }

            // Stop/reset: if Y/A was released while launcher was running (stage 2), OR if B is pressed on either gamepad
            if (((gamepad1.yWasReleased() || gamepad2.yWasReleased() || gamepad1.aWasReleased() || gamepad2.aWasReleased()) && launcherStage == 2)
                    || gamepad1.b || gamepad2.b) {
                // Stop the outtake wheel, reset launcher state, and lower transfer / stop intake
                // NEW -- should keep launcher active for faster shooting.
                if (!shouldKeepLauncherActive) OuttakeMotor.setVelocity(Constants.STOP_VELOCITY);
                launcherStage = 0;
                transferMotor.setPower(Constants.TRANSFER_DOWN_POSITION);
                intakeMotor.setPower(0);
                shouldAutoAlign = false; // don't auto align more
            }

            if (!(launcherStage==2)) {
                // Intake
                double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
                intakePower += gamepad2.right_trigger - gamepad2.left_trigger;
                intakePower = intakePower * 1.2;
                intakePower = Math.max(-1, Math.min(1, intakePower)); // Clip to [-1, 1]
                intakeMotor.setPower(intakePower);
                if (Math.abs(intakePower) >= 0.3) {
                    transferMotor.setPower(-0.8 * Math.abs(intakePower));
                } else {
                    transferMotor.setPower(0);
                }
            }

            // DPAD RESETS
            if (gamepad1.dpadDownWasPressed()) follower.resetHeading(0);
            //if (gamepad1.dpadLeftWasPressed()) shouldUseLimelightAutoAlign = !shouldUseLimelightAutoAlign;
            if (gamepad1.dpadUpWasPressed()) slowMode = !slowMode;
            if (gamepad1.dpadRightWasPressed()) shouldKeepLauncherActive = !shouldKeepLauncherActive;

            // ------------------------------------
            telemetry.addData("POSE", follower.getPose());
            telemetry.addData("LL Can see goal", autoAlignSystem.LLCanSeeGoal());
            telemetry.addData("RGB Indicator Color", rgbIndicator.getPWM() == RGBIndicator.GREEN_PWM ? "GREEN" : rgbIndicator.getPWM() == RGBIndicator.ORANGE_PWM ? "ORANGE" : rgbIndicator.getPWM() == RGBIndicator.YELLOW_PWM ? "YELLOW" : rgbIndicator.getPWM() == RGBIndicator.BLACK_PWM ? "BLACK" : rgbIndicator.getPWM() == RGBIndicator.RED_PWM ? "RED": ("UNKNOWN COLOR (PWM:" + rgbIndicator.getPWM() + ")"));
            telemetry.addData("Slow Mode", slowMode);
            telemetry.addData("Launcher Stage", launcherStage);
            telemetry.addData("Alliance Color", allianceColor);
            telemetry.addData("Intake Motor power", intakeMotor.getPower());
            telemetry.addData("Transfer Motor power", transferMotor.getPower());
            telemetry.addData("Outake Motor Velocity:", OuttakeMotor.getVelocity());
            telemetry.addData("Target Velocity", setTargetVelocity);
            telemetry.addData("Min Velocity", setMinVelocity);
            telemetry.addData("Tag ID", limelight.tid);
            telemetry.addData("Is fiducial null", fiducialResult==null);
            telemetry.addData("Distance", distance);
            telemetry.addData("Turning Power (RX)", rx);
            telemetry.update();
        }
    }

    public void handleTeleOpShootingUpdate(DcMotorEx OuttakeMotor, DcMotor intakeMotor, DcMotor transferMotor, double distance, boolean autoAlign) {
        switch (launcherStage) {
            case 0:
                if (autoAlign) shouldAutoAlign = true;
                setTargetVelocity = VelocityCalculator.NEWBOT.calculateVelocity(distance);
                if (Double.isNaN(setTargetVelocity)) setTargetVelocity = 1000;
                setMinVelocity = setTargetVelocity - Constants.VELOCITY_TOLERANCE;
                OuttakeMotor.setVelocity(setTargetVelocity);
                launcherStage = 1;
                launcherTimer.reset();
                break;
            case 1:
                double newSetVelocity = VelocityCalculator.NEWBOT.calculateVelocity(distance);
                if (Double.isNaN(newSetVelocity)) newSetVelocity = setTargetVelocity;
                double fusionWeight = 0.3; // use lowpass filter
                setTargetVelocity = setTargetVelocity*(1-fusionWeight) + newSetVelocity*fusionWeight;
                setMinVelocity = setTargetVelocity - Constants.VELOCITY_TOLERANCE;
                OuttakeMotor.setVelocity(setTargetVelocity);
                boolean speedOk = OuttakeMotor.getVelocity() >= setMinVelocity && (OuttakeMotor.getVelocity() <= setTargetVelocity+Constants.VELOCITY_TOLERANCE);
                boolean autoAlignOk = autoAlign && rgbIndicator.getPWM() == RGBIndicator.GREEN_PWM;
                // Add auto align semi-ok check + time out to prevent trying to auto align forever
                // if Aligning for more than 3 seconds, and stuck at deadzone, just shoot.
                boolean autoAlignSemiOk = autoAlign && limelight.getLLScore() < 4 && launcherTimer.seconds() > 3;
                if (speedOk && (!autoAlign || autoAlignOk || autoAlignSemiOk)) {
                    launcherStage = 2;
                    launcherTimer.reset();
                    if (autoAlign) shouldAutoAlign = false;
                }
                break;
            case 2:
                intakeMotor.setPower(1);
                transferMotor.setPower(Constants.TRANSFER_UP_POSITION);
                break;
            default:
                launcherStage = 0;
                launcherTimer.reset();
                break;
        }
    }
}