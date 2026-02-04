package org.firstinspires.ftc.teamcode2.TeleOp;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode2.Systems.Constants;
import org.firstinspires.ftc.teamcode2.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode2.Systems.RGBIndicator;


@TeleOp(name = "Era TeleOp", group = "TeleOp")// Name and Group
public class EraTeleOp extends LinearOpMode {

    final boolean TwoGamepads = false;
    //public LimelightSystem limelight;

    // declaring our PIDF tuning values
    double setTargetVelocity = 0;
    double setMinVelocity = 0;
   // private Follower follower;
    public static Pose startingPose;
    Constants.AllianceColor allianceColor;
    LimelightSystem limelight;

    RGBIndicator rgbIndicator;

    @Override
    public void runOpMode() throws InterruptedException {
        // For now - don't pass in hardware Map because then it won't throw an error.
        // When Limelight is added, pass in hardwareMap
        limelight = new LimelightSystem();
        //  rgbIndicator = new RGBIndicator(hardwareMap.get(Servo.class, "rgb"));
        //    rgbIndicator.setColor(RGBIndicator.Color.VIOLET);
        //while (!isStarted() && !isStopRequested())
        while (opModeInInit())
        {
            telemetry.addData("Press 'GamePad1 Right Bumper'", "for BLUE");
            telemetry.addData("Press 'GamePad1 Left Bumper'", "for RED");
            // This method is called repeatedly during the init phase
            allianceColor = Constants.AllianceColor.RED;
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
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("flm");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("blm");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frm");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("brm");
        // Using DcMotorEx instead of DcMotor to use PID controller
        DcMotorEx OuttakeMotor = hardwareMap.get(DcMotorEx.class,"launcher");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intake");
        DcMotor transferMotor = hardwareMap.dcMotor.get("transfer");

        //follower = Constants.createPedroFollower(hardwareMap);

        telemetry.addLine("Setting the motors now");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Here we set our Left and Right Outtake Motor to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        OuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        OuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        OuttakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.LaunchPIDF);

        if (allianceColor == Constants.AllianceColor.RED)
        {
            // Starting position Red Goal
            startingPose = new Pose();
            //follower.setStartingPose(startingPose);
            //follower.update();
            telemetry.addData("Alliance Color", "Red");
            //telemetry.addData("Starting Pose", follower.getPose());
        }
        else if(allianceColor == Constants.AllianceColor.BLUE)
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
          //  telemetry.addData("Starting Pose", follower.getPose());
            telemetry.addData("Alliance Color Variable", allianceColor);
        }

        //telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        /*
         * Tell the driver that initialization is complete.
         */

        telemetry.addData("Status", "Initialized");telemetry.update();
        //      limelight.setDevice(hardwareMap.get(Limelight3A.class, "limelight"));
        waitForStart();
        //   limelight.startLLWithPipeline(0);
        //   follower.startTeleopDrive();

        if (isStopRequested()) return;
        telemetry.addData("Status", "Running");
        while (opModeIsActive()) {
            /*
            limelight.LLUpdate();
            telemetry.addData("Limelight Score", limelight.getLLScore());
            if (limelight.getLLScore() == 0) rgbIndicator.setColor(RGBIndicator.Color.VIOLET);
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

             */
            // if (limelight.isDisconnected) rgbIndicator.setColor(RGBIndicator.Color.RED);telemetry.addData("Disconnected",""); // DISCONNECTED
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

           // follower.update();
//            telemetryM.update();
            /*
           if (!automatedDrive)
            {
                if (!slowMode) follower.setTeleOpDrive(
                       -gamepad1.left_stick_y, // Remember, Y stick value is reversed
                        gamepad1.left_stick_x * 1.1,
                       gamepad1.right_stick_x,
                       true // Robot Centric
               );
            }
            */
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = 1 *(y + x + rx) / denominator;
            double backLeftPower = 1 *(y - x + rx) / denominator;
            double frontRightPower = 1 *(y - x - rx) / denominator;
            double backRightPower = 1 * (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            /*
             * LL:
             * * ------- gamepad2.a:
             * Step 1. Find the heading and rotate to face the goal
             * Step 2. Move forward/backward so that the robot is the correct distance from the goal
             * Step 3. Start the outtake motors
             * ------- dpad_up:
             * Step 4. Check if the velocity of the motors is more than the min velocity
             * Step 5. position servo into launch position
             * NORMAL:
             * When driver presses the button to launch, 7 things need to happen
             * Step 1. Find the position of the robot on the field
             * Step 2. Determine the TARGET_VELOCITY based on robot position
             * Step 3. Determine the current heading of the robot in the field
             * Step 4. Change direction of the robot so its aimed at the goal
             * Step 5. Set TARGET_VELOCITY determined in Step 2 to start the left and right outake motors
             * Step 6. Check if the velocity of the motors is more than the min velocity
             * Step 7. position servo into launch position
             */

            if (gamepad1.y) {
                setMinVelocity = Constants.TARGET_VELOCITY_FRONT_LAUNCH_ZONE - Constants.VELOCITY_TOLERANCE;
                setTargetVelocity = Constants.TARGET_VELOCITY_FRONT_LAUNCH_ZONE;
                OuttakeMotor.setVelocity(setMinVelocity);
                OuttakeMotor.setVelocity(setTargetVelocity);

            }

            if (gamepad1.a)
            {
                setMinVelocity = Constants.TARGET_VELOCITY_FRONT_LAUNCH_ZONE - Constants.VELOCITY_TOLERANCE;
                setTargetVelocity = Constants.TARGET_VELOCITY_BACK_LAUNCH_ZONE;
                OuttakeMotor.setVelocity(setMinVelocity);
                OuttakeMotor.setVelocity(setTargetVelocity);

            }

            if (gamepad1.x) {
                intakeMotor.setPower(1);
                transferMotor.setPower(0.5);
            }

            // In-take
            if (gamepad1.dpad_right) {
                intakeMotor.setPower(-1);
            } else {
                //intakeMotor.setPower(0);
            }

            if (gamepad1.b) {
                OuttakeMotor.setVelocity(Constants.STOP_VELOCITY);
                transferMotor.setPower(0);
                setTargetVelocity = 0;
                setMinVelocity = 0;
            }

            if (gamepad1.rightBumperWasPressed()) {
                setTargetVelocity += Constants.INCREMENT_CHANGE_IN_VELOCITY;
                OuttakeMotor.setVelocity(setTargetVelocity);
            }

            if (gamepad1.leftBumperWasPressed()) {
                if (setTargetVelocity > Constants.INCREMENT_CHANGE_IN_VELOCITY) {
                    setTargetVelocity -= Constants.INCREMENT_CHANGE_IN_VELOCITY;
                    OuttakeMotor.setVelocity(setTargetVelocity);
                }
            }

            // Reverse In-take Send balls out of the motor, opposite of Intake
            if (gamepad1.dpad_left) {
                intakeMotor.setPower(1);
            } else {
                intakeMotor.setPower(0);
            }

            telemetry.addData("Intake Motor power", intakeMotor.getPower());
            telemetry.addData("Transfer Motor power", transferMotor.getPower());
            telemetry.addData("Outake Motor Velocity:", OuttakeMotor.getVelocity());
            telemetry.addData("Target Velocity", setTargetVelocity);
            telemetry.addData("Min Velocity", setMinVelocity);
            telemetry.update();
        }
    }
}