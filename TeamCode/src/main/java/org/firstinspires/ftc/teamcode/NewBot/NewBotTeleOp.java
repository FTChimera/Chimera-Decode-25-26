package org.firstinspires.ftc.teamcode.NewBot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode.Systems.RGBIndicator;


@SuppressWarnings("SpellCheckingInspection")
@TeleOp(name = "NewBotTeleOp", group = "TeleOp")// Name and Group
public class NewBotTeleOp extends LinearOpMode {

    boolean Gamepad2Driving = false, launcherOn= false;
    //public LimelightSystem limelight;

    // declaring our PIDF tuning values
    double setTargetVelocity = 0;
    double setMinVelocity = 0;
    // private Follower follower;
    public static Pose startingPose;
    Constants.AllianceColor allianceColor;
    LimelightSystem limelight;
    AutoAlignSystem autoAlignSystem;

    RGBIndicator rgbIndicator;

    @Override
    public void runOpMode() throws InterruptedException {
        // For now - don't pass in hardware Map because then it won't throw an error.
        // When Limelight is added, pass in hardwareMap
        limelight = new LimelightSystem(hardwareMap);
        rgbIndicator = new RGBIndicator(hardwareMap);
        rgbIndicator.setColor(RGBIndicator.Color.VIOLET);
        //while (!isStarted() && !isStopRequested())
        allianceColor = Constants.AllianceColor.RED;
        while (opModeInInit())
        {
            telemetry.addData("Press 'GamePad1 Right Bumper'", "for BLUE");
            telemetry.addData("Press 'GamePad1 Left Bumper'", "for RED");
            telemetry.addLine("Press A for Start A Driver, B for Start B Driver");
            telemetry.addLine(Gamepad2Driving ? "Gamepad B" : "Gamepad A" + " Driving");
            // This method is called repeatedly during the init phase
            if (gamepad1.a) {
                Gamepad2Driving = false;
            }
            if (gamepad1.b) {
                Gamepad2Driving = true;
            }
            if (gamepad1.right_bumper)
            {
                allianceColor = Constants.AllianceColor.BLUE;
                // Display the current selection on the Driver Station
                telemetry.addData("Alliance", "Selected: ", "BLUE");
            } else if (gamepad1.left_bumper) {
                allianceColor = Constants.AllianceColor.RED;
                // Display the current selection on the Driver Station
                telemetry.addData("Alliance", "Selected: ", "RED");
            } else {
                allianceColor = Constants.AllianceColor.RED;
                // Display the current selection on the Driver Station
                telemetry.addData("Alliance", "Selected: ", "RED");
            }
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

        //follower = Constants.createPedroFollower(hardwareMap);

        telemetry.addLine("Setting the motors now");

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        OuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        OuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        OuttakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.LaunchPIDF);

        if (allianceColor == Constants.AllianceColor.RED)
        {
            autoAlignSystem = new AutoAlignSystem(Constants.AllianceColor.RED);
            // Starting position Red Goal
            startingPose = new Pose();
            //follower.setStartingPose(startingPose);
            //follower.update();
            telemetry.addData("Alliance Color", "Red");
            //telemetry.addData("Starting Pose", follower.getPose());
        }
        else if(allianceColor == Constants.AllianceColor.BLUE)
        {
            autoAlignSystem = new AutoAlignSystem(Constants.AllianceColor.BLUE);
            startingPose = new Pose();
            //follower.setStartingPose(startingPose);
            //follower.update();
            telemetry.addData("Alliance Color", "Blue");
            //telemetry.addData("Starting Pose", follower.getPose());
        } else {
            autoAlignSystem = new AutoAlignSystem(Constants.AllianceColor.RED);
            // Starting position Red Goal
            startingPose = new Pose();
            //follower.setStartingPose(startingPose);
            // follower.update();
            telemetry.addData("Alliance Color", "Red");
            // telemetry.addData("Starting Pose", follower.getPose());
            telemetry.addData("Alliance Color Variable", allianceColor);
        }

        autoAlignSystem.LimelightSetUp(limelight, new DcMotor[]{frontLeftMotor,backLeftMotor, frontRightMotor, backRightMotor});

        //telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        /*
         * Tell the driver that initialization is complete.
         */

        telemetry.addData("Status", "Initialized");telemetry.update();
        // limelight.setDevice(hardwareMap.get(Limelight3A.class, "limelight"));
        waitForStart();
        // limelight.startLLWithPipeline(0);
        // follower.startTeleopDrive();

        if (isStopRequested()) return;
        limelight.start(0);
        telemetry.addData("Status", "Running");

        // --- NEW VARIABLES FOR BUTTON TOGGLES ---
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;
        // ----------------------------------------

        // calculate dt for turning with limelight
        long lastTime = System.nanoTime();
        long currentTime;
        double deltaTime;

        while (opModeIsActive()) {

            limelight.LLUpdate();
            telemetry.addData("Limelight Score", limelight.getLLScore());
            telemetry.addData("Limelight TagID", limelight.tid); // debug: show which tag limelight reports
            rgbIndicator.updateUsingLL(limelight);


            double y, x, rx;
            if (Gamepad2Driving) {
                y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
                x = gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
                rx = gamepad2.right_stick_x;
            } else {
                y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                rx = gamepad1.right_stick_x;
            }

            y *= y*y;
            x *= x*x;
            rx *= rx*rx * 0.75;

            // Calculate dt
            currentTime = System.nanoTime();
            deltaTime = (currentTime - lastTime) / 1e9; // convert to seconds
            lastTime = currentTime; // Update lastTime each loop so dt is meaningful


            if (gamepad1.back || gamepad2.back) {
                rx = autoAlignSystem.getTurningPowerLimelight(deltaTime);
                telemetry.addData("rx", rx);
            }
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = 0.9 *(y + x + rx) / denominator;
            double backLeftPower = 0.9 *(y - x + rx) / denominator;
            double frontRightPower = 0.9 *(y - x - rx) / denominator;
            double backRightPower = 0.9 * (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            if (gamepad1.y || gamepad2.y) {
                launcherOn = true;
                //setTargetVelocity = Constants.TARGET_VELOCITY_FRONT_LAUNCH_ZONE;
                setTargetVelocity = VelocityCalculator.NEWBOT.calculateVelocity(limelight.dist); // use new velocity calculator
                setMinVelocity = setTargetVelocity - Constants.VELOCITY_TOLERANCE;
                OuttakeMotor.setVelocity(setMinVelocity);
                OuttakeMotor.setVelocity(setTargetVelocity);

            }

//            if (gamepad1.a || gamepad2.a)
//            {
//                launcherOn = true;
//                setTargetVelocity = Constants.TARGET_VELOCITY_BACK_LAUNCH_ZONE;
//                setMinVelocity = setTargetVelocity - Constants.VELOCITY_TOLERANCE;
//                OuttakeMotor.setVelocity(setMinVelocity);
//                OuttakeMotor.setVelocity(setTargetVelocity);
//
//            }

            if ((gamepad1.x || gamepad2.x) && launcherOn) {
                intakeMotor.setPower(1);
                transferMotor.setPower(Constants.TRANSFER_UP_POSITION);
            } else {

                // In-take
                double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
                intakePower += gamepad2.right_trigger - gamepad2.left_trigger;
                intakePower = intakePower * 1.5;
                intakeMotor.setPower(intakePower);
                if (!(launcherOn)) {
                    if (Math.abs(intakePower) >= 0.3) {
                        transferMotor.setPower(-0.5*Math.abs(intakePower));
                    } else {
                        transferMotor.setPower(0);
                    }
                }
            }
            if (gamepad1.b || gamepad2.b) {
                launcherOn = false;
                OuttakeMotor.setVelocity(Constants.STOP_VELOCITY);
                transferMotor.setPower(Constants.TRANSFER_DOWN_POSITION);
                setTargetVelocity = 0;
                setMinVelocity = 0;
            }

            // Check for Dpad Up (Increase Velocity)
            boolean currentDpadUp = gamepad1.dpad_up || gamepad2.dpad_up;
            if (currentDpadUp && !lastDpadUp) {
                setTargetVelocity += 25;
                setMinVelocity = setTargetVelocity - Constants.VELOCITY_TOLERANCE;
                OuttakeMotor.setVelocity(setTargetVelocity);
            }
            lastDpadUp = currentDpadUp;

            // Check for Dpad Down (Decrease Velocity)
            boolean currentDpadDown = gamepad1.dpad_down || gamepad2.dpad_down;
            if (currentDpadDown && !lastDpadDown) {
                if (setTargetVelocity > 25) {
                    setTargetVelocity -= 25;
                    setMinVelocity = setTargetVelocity - Constants.VELOCITY_TOLERANCE;
                    OuttakeMotor.setVelocity(setTargetVelocity);
                }
            }
            lastDpadDown = currentDpadDown;

            // ------------------------------------

            telemetry.addData("Intake Motor power", intakeMotor.getPower());
            telemetry.addData("Transfer Motor power", transferMotor.getPower());
            telemetry.addData("Outake Motor Velocity:", OuttakeMotor.getVelocity());
            telemetry.addData("Target Velocity", setTargetVelocity);
            telemetry.addData("Min Velocity", setMinVelocity);
            telemetry.addData("Dist (1/ta)", limelight.dist);
            telemetry.update();
        }
    }
}