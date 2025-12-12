package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Systems.Consts;
import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode.Systems.RGBIndicator;
import org.firstinspires.ftc.teamcode.pedroAuto.Constants;


@TeleOp(name = "ChimeraTeleOpBadOldNoConst", group = "AbsolutePriority")// Name and Group
public class ChimeraTeleOpBadOldNoConst extends LinearOpMode {

    final double TARGET_VELOCITY_BACK_LAUNCH_ZONE = 1150;// Set target velocity from back launch zone
    final double TARGET_VELOCITY_FRONT_LAUNCH_ZONE = 900;// Set target velocity from front launch zone
    final double MIN_VELOCITY_BACK_LAUNCH_ZONE = 1050;// Set target velocity from back launch zone
    final double MIN_VELOCITY_FRONT_LAUNCH_ZONE = 100;// Set target velocity from back launch zone
    final double STOP_VELOCITY = 0; // Set target velocity- in RPM(e.g., 3000 RPM)
    final int SERVO_LAUNCH_POSITION = 0;
    final int SERVO_REST_POSITION = 1;
    final int SLEEP_BEFORE_RESET_SERVO_POSITION = 200;
    public LimelightSystem.ChimeraLL limelight = new LimelightSystem.ChimeraLL();

    // declaring our PIDF tuning values

    double  setTargetVelocity = 0;
    double setMinVelocity = 0;
    public static Pose startingPose;
    private Follower follower;
    enum AllianceColor {
        BLUE,
        RED
    };
    AllianceColor allianceColor;

    ElapsedTime feederTimer = new ElapsedTime();
    RGBIndicator rgbIndicator;

    @Override
    public void runOpMode() throws InterruptedException {
        rgbIndicator = new RGBIndicator(hardwareMap.get(Servo.class, "rgb"));
        rgbIndicator.setColor(RGBIndicator.Color.VIOLET);
        //while (!isStarted() && !isStopRequested())
        while (opModeInInit())
        {
            telemetry.addData("Press 'GamePad1 Right Bumper'", "for BLUE");
            telemetry.addData("Press 'GamePad1 Left Bumper'", "for RED");
            // This method is called repeatedly during the init phase
            allianceColor = AllianceColor.RED;
            if (gamepad1.right_bumper)
            {
                allianceColor = AllianceColor.BLUE;
                // Display the current selection on the Driver Station
                telemetry.addData("Alliance", "Selected: ", "BLUE");
            } else if (gamepad1.left_bumper) {
                allianceColor = AllianceColor.RED;
                // Display the current selection on the Driver Station
                telemetry.addData("Alliance", "Selected: ", "RED");
            } else {
                allianceColor = AllianceColor.RED;
                // Display the current selection on the Driver Station
                telemetry.addData("Alliance", "Selected: ", "RED");
            }
            telemetry.update();
        }

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        // Using DcMotorEx instead of DcMotor to use PID controller
        DcMotorEx rightOutakeMotor = hardwareMap.get(DcMotorEx.class,"OutakeMotorRight");
        DcMotorEx leftOutakeMotor = hardwareMap.get(DcMotorEx.class,"OutakeMotorLeft");

        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        Servo pushServo = hardwareMap.servo.get("pushServo");

        follower = Constants.createFollower(hardwareMap);


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightOutakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftOutakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        /*
         * Here we set our Left and Right Outtake Motor to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        leftOutakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOutakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        rightOutakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftOutakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftOutakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Consts.leftPIDF);
        rightOutakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Consts.rightPIDF);
        if (allianceColor == AllianceColor.RED)
        {
            // Starting position Red Goal
            startingPose = Consts.RED_STARTING_POSE;
            follower.setStartingPose(startingPose);
            follower.update();
            telemetry.addData("Alliance Color", "Red");
            telemetry.addData("Starting Pose", follower.getPose());
        }
        else if(allianceColor == AllianceColor.BLUE)
        {
            startingPose = Consts.BLUE_STARTING_POSE;
            follower.setStartingPose(startingPose);
            follower.update();
            telemetry.addData("Alliance Color", "Blue");
            telemetry.addData("Starting Pose", follower.getPose());
        } else {
            // Starting position Red Goal
            startingPose = Consts.RED_STARTING_POSE;
            follower.setStartingPose(startingPose);
            follower.update();
            telemetry.addData("Alliance Color", "Red");
            telemetry.addData("Starting Pose", follower.getPose());
            telemetry.addData("Alliance Color Variable", allianceColor);
        }

        //telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        /*
         * Tell the driver that initialization is complete.
         */

        telemetry.addData("Status", "Initialized");telemetry.update();
        limelight.setDevice(hardwareMap.get(Limelight3A.class, "limelight"));
        waitForStart();
        limelight.startLLWithPipeline(0);
        follower.startTeleopDrive();

        if (isStopRequested()) return;
        telemetry.addData("Status", "Running");
        while (opModeIsActive()) {
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
            // if (limelight.isDisconnected) rgbIndicator.setColor(RGBIndicator.Color.RED);telemetry.addData("Disconnected",""); // DISCONNECTED
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            follower.update();
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


            if (gamepad2.a)
            {
                // Step 2. Determine the TARGET_VELOCITY based on robot position
                // Calculate distance from robot to the goal
                /*
                if (allianceColor == AllianceColor.RED) {
                    Distance_To_Goal = Math.sqrt(Math.pow((X_Coordinate_Red_Goal - X_Coordinate), 2) + Math.pow((Y_Coordinate_Red_Goal - Y_Coordinate), 2));
                } else if (allianceColor == AllianceColor.BLUE)
                {
                    Distance_To_Goal = Math.sqrt(Math.pow((X_Coordinate_Blue_Goal - X_Coordinate), 2) + Math.pow((Y_Coordinate_Blue_Goal - Y_Coordinate), 2));
                }
                telemetry.addData("Distance to Goal", Distance_To_Goal);

                if (Distance_To_Goal <= SHOOTING_ZONE_CLOSE_FRONT_LAUNCH_ZONE)
                {
                    setMinVelocity = MIN_VELOCITY_FRONT_LAUNCH_ZONE;
                    setTargetVelocity = TARGET_VELOCITY_FRONT_LAUNCH_ZONE;
                }
                else if ((Distance_To_Goal > SHOOTING_ZONE_CLOSE_FRONT_LAUNCH_ZONE) &&
                        ( Distance_To_Goal <= SHOOTING_ZONE_FAR_FRONT_LAUNCH_ZONE))
                {
                    // TODO: Change these values
                    setMinVelocity = MIN_VELOCITY_FRONT_LAUNCH_ZONE;
                    setTargetVelocity = TARGET_VELOCITY_FRONT_LAUNCH_ZONE;

                }
                else if(Distance_To_Goal > SHOOTING_ZONE_FAR_FRONT_LAUNCH_ZONE)
                {
                    setMinVelocity = MIN_VELOCITY_BACK_LAUNCH_ZONE;
                    setTargetVelocity = TARGET_VELOCITY_BACK_LAUNCH_ZONE;
                }
                //  Step 3. Determine the current heading of the robot in the field
                if (allianceColor == AllianceColor.RED)
                {
                    //Math.atan2 returns the angle in radians
                    launchPositionHeadingRadians = Math.atan2( (Y_Coordinate_Red_Goal - Y_Coordinate), (X_Coordinate_Red_Goal - X_Coordinate) );
                } else if(allianceColor == AllianceColor.BLUE)
                {
                    //Math.atan2 returns the angle in radians
                    launchPositionHeadingRadians = Math.atan2( (Y_Coordinate_Blue_Goal - Y_Coordinate), (X_Coordinate_Blue_Goal - X_Coordinate) );
                }


                // Step 4. Change direction of the robot so its aimed at the goal
                Pose scorePose = new Pose(X_Coordinate, Y_Coordinate, launchPositionHeadingRadians); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
                follower.setPose(scorePose);
                follower.update();
                telemetry.addData("Current and Score Pose X Coordinate", X_Coordinate);
                telemetry.addData("Current and Score Pose Y Coordinate", Y_Coordinate);
                telemetry.addData("Current heading", Math.toDegrees(currentHeading));
                telemetry.addData("Score Pose heading in Degrees", Math.toDegrees(launchPositionHeadingRadians));
                telemetry.addData("Score Pose heading in Radians", launchPositionHeadingRadians);
                telemetry.addData("Target Velocity", setTargetVelocity);
                telemetry.addData("Min Velocity", setMinVelocity);
                */

                // Step 5. Set TARGET_VELOCITY determined in Step 2 to start the left and right outake motors

                setMinVelocity = MIN_VELOCITY_BACK_LAUNCH_ZONE;
                setTargetVelocity = TARGET_VELOCITY_BACK_LAUNCH_ZONE;
                leftOutakeMotor.setVelocity(setTargetVelocity);
                rightOutakeMotor.setVelocity(setTargetVelocity);
                pushServo.setPosition(SERVO_REST_POSITION);

                telemetry.addData("Left Outake Motor Velocity", leftOutakeMotor.getVelocity());
                telemetry.addData("Right Outake Motor Velocity", rightOutakeMotor.getVelocity());
                //telemetry.update();
                // Step 6 and Step 7 are performed upon pressing dpad_up.
            }

            // Step 6. Check if the velocity of the motors is more than the min velocity
            if (gamepad2.dpad_up)
            {
                pushServo.setPosition(SERVO_REST_POSITION);
                telemetry.addData("Launch: Left Outake Motor Velocity", leftOutakeMotor.getVelocity());
                telemetry.addData("Launch: Right Outake Motor Velocity", rightOutakeMotor.getVelocity());
                telemetry.addData("Launch: Min Velocity ", setMinVelocity);
                pushServo.setPosition(SERVO_LAUNCH_POSITION);
                telemetry.addData("Launch: Setting Servo to Launch Position", "true");
                sleep(SLEEP_BEFORE_RESET_SERVO_POSITION);
                telemetry.addData("Launch: Sleeping", "true");
                pushServo.setPosition(SERVO_REST_POSITION);
                telemetry.addData("Launch: Setting Servo to Rest Position", "true");

                if ((leftOutakeMotor.getVelocity() >= setMinVelocity) && (rightOutakeMotor.getVelocity() >= setMinVelocity))
                {
                    //Step 7. position servo into launch position
                    pushServo.setPosition(SERVO_LAUNCH_POSITION);
                    telemetry.addData("Launch: Setting Servo to Launch Position", "true");
                    sleep(SLEEP_BEFORE_RESET_SERVO_POSITION);
                    telemetry.addData("Launch: Sleeping", "true");
                    pushServo.setPosition(SERVO_REST_POSITION);
                    telemetry.addData("Launch: Setting Servo to Rest Position", "true");
                }


            }

            if (gamepad2.b) {
                leftOutakeMotor.setVelocity(STOP_VELOCITY);
                rightOutakeMotor.setVelocity(STOP_VELOCITY);
                pushServo.setPosition(SERVO_REST_POSITION);
            }


            if (gamepad2.x){
                intakeMotor.setPower(1);
            } else if (!gamepad2.x) {
                intakeMotor.setPower(0);
            }


            if(gamepad2.y) {
                setMinVelocity = MIN_VELOCITY_FRONT_LAUNCH_ZONE;
                setTargetVelocity = TARGET_VELOCITY_FRONT_LAUNCH_ZONE;

                leftOutakeMotor.setVelocity(setTargetVelocity);
                rightOutakeMotor.setVelocity(setTargetVelocity);
                pushServo.setPosition(SERVO_REST_POSITION);
                telemetry.addData("Left Outake Motor Velocity Front:", leftOutakeMotor.getVelocity());
                telemetry.addData("Right Outake Motor Velocity Front:", rightOutakeMotor.getVelocity());
                telemetry.addData("Target Velocity front", setTargetVelocity);
                telemetry.addData("Min Velocity front", setMinVelocity);
            }


            /*
            if(gamepad2.dpad_up){
                pushServo.setPosition(0);

            }
            if(gamepad2.dpad_down){
                pushServo.setPosition(1);

            }
            */


            // telemetry.addData("Right Slide Position", rightViperSlide.getCurrentPosition());
            // telemetry.addData("Left Slide Position", leftViperSlide.getCurrentPosition());
            // telemetry.addData("Slide Increment", slidePosition);

            telemetry.update();
        }
    }
}