package org.firstinspires.ftc.teamcode.NewBot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode.Systems.RGBIndicator;


@TeleOp(name = "NewBotTeleOp", group = "TeleOp")// Name and Group
public class NewBotTeleOp extends LinearOpMode {

    boolean TwoGamepads = false;
    //public LimelightSystem limelight;

    // declaring our PIDF tuning values
    double setTargetVelocity = 0;
    double setMinVelocity = 0;
    // private Follower follower;
    public static Pose startingPose;
    ConstantsTeleOp.AllianceColor allianceColor;
    LimelightSystem limelight;

    RGBIndicator rgbIndicator;

    @Override
    public void runOpMode() throws InterruptedException {
        // For now - don't pass in hardware Map because then it won't throw an error.
        // When Limelight is added, pass in hardwareMap
        limelight = new LimelightSystem(hardwareMap);
        rgbIndicator = new RGBIndicator(hardwareMap);
        rgbIndicator.setColor(RGBIndicator.Color.VIOLET);
        //while (!isStarted() && !isStopRequested())
        allianceColor = ConstantsTeleOp.AllianceColor.RED;
        while (opModeInInit())
        {
            telemetry.addData("Press 'GamePad1 Right Bumper'", "for BLUE");
            telemetry.addData("Press 'GamePad1 Left Bumper'", "for RED");
            telemetry.addLine("Press A for 2 Gamepads, B for 1");
            // This method is called repeatedly during the init phase
            if (gamepad1.a) {
                TwoGamepads = true;
            }
            if (gamepad1.b) {
                TwoGamepads = false;
            }
            if (gamepad1.right_bumper)
            {
                allianceColor = ConstantsTeleOp.AllianceColor.BLUE;
                // Display the current selection on the Driver Station
                telemetry.addData("Alliance", "Selected: ", "BLUE");
            } else if (gamepad1.left_bumper) {
                allianceColor = ConstantsTeleOp.AllianceColor.RED;
                // Display the current selection on the Driver Station
                telemetry.addData("Alliance", "Selected: ", "RED");
            } else {
                allianceColor = ConstantsTeleOp.AllianceColor.RED;
                // Display the current selection on the Driver Station
                telemetry.addData("Alliance", "Selected: ", "RED");
            }
            telemetry.update();
        }

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get(Constants.driveConstants.leftFrontMotorName);
        DcMotor backLeftMotor = hardwareMap.dcMotor.get(Constants.driveConstants.leftRearMotorName);
        DcMotor frontRightMotor = hardwareMap.dcMotor.get(Constants.driveConstants.rightFrontMotorName);
        DcMotor backRightMotor = hardwareMap.dcMotor.get(Constants.driveConstants.rightRearMotorName);
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

        OuttakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ConstantsTeleOp.LaunchPIDF);

        if (allianceColor == ConstantsTeleOp.AllianceColor.RED)
        {
            // Starting position Red Goal
            startingPose = new Pose();
            //follower.setStartingPose(startingPose);
            //follower.update();
            telemetry.addData("Alliance Color", "Red");
            //telemetry.addData("Starting Pose", follower.getPose());
        }
        else if(allianceColor == ConstantsTeleOp.AllianceColor.BLUE)
        {
            startingPose = new Pose();
            //follower.setStartingPose(startingPose);
            //follower.update();
            telemetry.addData("Alliance Color", "Blue");
            //telemetry.addData("Starting Pose", follower.getPose());
        } else {
            // Starting position Red Goal
            startingPose = new Pose();
            //follower.setStartingPose(startingPose);
            // follower.update();
            telemetry.addData("Alliance Color", "Red");
            // telemetry.addData("Starting Pose", follower.getPose());
            telemetry.addData("Alliance Color Variable", allianceColor);
        }

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

        while (opModeIsActive()) {

            limelight.LLUpdate();
            telemetry.addData("Limelight Score", limelight.getLLScore());
            if (limelight.getLLScore() == 0) rgbIndicator.setColor(RGBIndicator.Color.BLACK);
            else if (limelight.getLLScore() < 6) {
                // GREEN
                rgbIndicator.setColor(RGBIndicator.Color.GREEN);
            } else if (limelight.getLLScore() < 10) {
                // ORANGE
                rgbIndicator.setColor(RGBIndicator.Color.GOLD);
            } else {
                // OFF
                rgbIndicator.setColor(RGBIndicator.Color.BLACK);
            }


            if (limelight.isDisconnected) rgbIndicator.setColor(RGBIndicator.Color.RED);telemetry.addData("Disconnected",""); // DISCONNECTED
            double y, x, rx;
            if (TwoGamepads) {
                y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
                x = gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
                rx = gamepad2.right_stick_x;
            } else {
                y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                rx = gamepad1.right_stick_x;
            }

            y *= Math.abs(y);
            x *= Math.abs(x);
            rx *= Math.abs(rx);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = 1 *(y + x + rx) / denominator;
            double backLeftPower = 1 *(y - x + rx) / denominator;
            double frontRightPower = 1 *(y - x - rx) / denominator;
            double backRightPower = 1 * (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            if (gamepad1.y) {
                setMinVelocity = ConstantsTeleOp.TARGET_VELOCITY_FRONT_LAUNCH_ZONE - ConstantsTeleOp.VELOCITY_TOLERANCE;
                setTargetVelocity = ConstantsTeleOp.TARGET_VELOCITY_FRONT_LAUNCH_ZONE;
                OuttakeMotor.setVelocity(setMinVelocity);
                OuttakeMotor.setVelocity(setTargetVelocity);

            }

            if (gamepad1.a)
            {
                setMinVelocity = ConstantsTeleOp.TARGET_VELOCITY_FRONT_LAUNCH_ZONE - ConstantsTeleOp.VELOCITY_TOLERANCE;
                setTargetVelocity = ConstantsTeleOp.TARGET_VELOCITY_BACK_LAUNCH_ZONE;
                OuttakeMotor.setVelocity(setMinVelocity);
                OuttakeMotor.setVelocity(setTargetVelocity);

            }

            if (gamepad1.x) {
                intakeMotor.setPower(1);
                transferMotor.setPower(ConstantsTeleOp.TRANSFER_UP_POSITION);
            } else {

                // In-take
                double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
                intakePower = intakePower * 1.5;
                intakeMotor.setPower(intakePower);
            }
            if (gamepad1.b) {
                OuttakeMotor.setVelocity(ConstantsTeleOp.STOP_VELOCITY);
                transferMotor.setPower(ConstantsTeleOp.TRANSFER_DOWN_POSITION);
                setTargetVelocity = 0;
                setMinVelocity = 0;
            }

            // Check for Dpad Up (Increase Velocity)
            boolean currentDpadUp = gamepad1.dpad_up;
            if (currentDpadUp && !lastDpadUp) {
                setTargetVelocity += 25;
                OuttakeMotor.setVelocity(setTargetVelocity);
            }
            lastDpadUp = currentDpadUp;

            // Check for Dpad Down (Decrease Velocity)
            boolean currentDpadDown = gamepad1.dpad_down;
            if (currentDpadDown && !lastDpadDown) {
                if (setTargetVelocity > 25) {
                    setTargetVelocity -= 25;
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
            telemetry.update();
        }
    }
}