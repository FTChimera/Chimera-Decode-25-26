package org.firstinspires.ftc.teamcode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Systems.Consts;
import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode.Systems.RGBIndicator;
import org.firstinspires.ftc.teamcode.pedroAuto.Constants;


@TeleOp(name = "ChimeraTeleOp",group = "AbsolutePriority")// Name and Group
public class ChimeraTeleOpLimeLight extends LinearOpMode {
    public LimelightSystem.ChimeraLL limelight = new LimelightSystem.ChimeraLL();
    double  setTargetVelocity;
    double setMinVelocity = 0;
    private Follower follower;
    public static Pose startingPose;

    // TODO Change Starting position. Temporarily set starting position to back launch
    // zone, (x,y) = (72,0)
    Consts.AllianceColor allianceColor;
    RGBIndicator rgbIndicator;
    boolean OneGamepadAControl;

    public double findAngleToRotate() {
        // return allianceColor==Consts.AllianceColor.RED? Math.atan2( (Consts.Y_Coordinate_Red_Goal - curPose.getY()), (Consts.X_Coordinate_Red_Goal - curPose.getX()) ):Math.atan2( (Consts.Y_Coordinate_Blue_Goal - curPose.getY()), (Consts.X_Coordinate_Blue_Goal - curPose.getX()) );
        return Math.toRadians(limelight.tx);
    }
    public Path getPath(double amt) {
        // THIS CODE MAKES THE ROBOT MOVE FORWARD/BACKWARD
        Pose pose = follower.getPose();
        double heading = pose.getHeading() - (allianceColor==Consts.AllianceColor.RED?0.79:-0.79);
        double RobotCentric_X = Math.cos(heading);
        double RobotCentric_Y = Math.sin(heading);
        Pose endPose = new Pose(pose.getX() + RobotCentric_X*amt, pose.getY() + RobotCentric_Y*amt, heading);
        Path path = new Path(new BezierLine(pose, endPose));path.setLinearHeadingInterpolation(heading,heading);
        return path;
    }
    public void rotate(double angle_Radians) {follower.turn(Math.abs(angle_Radians), angle_Radians/Math.abs(angle_Radians)==-1);follower.update();}
    public void LLRunGamepadA(DcMotorEx leftOutakeMotor, DcMotorEx rightOutakeMotor, Servo pushServo){
        // STEP 1
        rotate(findAngleToRotate() * Consts.LAUNCHER_GOALTAG_ANGLE_SCALE);
        // STEP 2
        follower.followPath(getPath(limelight.dist));follower.update();
        // STEP 3
        setMinVelocity = Consts.MIN_VELOCITY_BACK_LAUNCH_ZONE;setTargetVelocity = Consts.TARGET_VELOCITY_BACK_LAUNCH_ZONE;
        leftOutakeMotor.setVelocity(setTargetVelocity);rightOutakeMotor.setVelocity(setTargetVelocity);
        pushServo.setPosition(Consts.SERVO_REST_POSITION);
    }
    public void LLRunDpadUp(DcMotorEx leftOutakeMotor, DcMotorEx rightOutakeMotor, Servo pushServo){
        telemetry.addData("Launch: Left Outake Motor Velocity", leftOutakeMotor.getVelocity());
        telemetry.addData("Launch: Right Outake Motor Velocity", rightOutakeMotor.getVelocity());
        telemetry.addData("Launch: Min Velocity ", setMinVelocity);
        pushServo.setPosition(Consts.SERVO_LAUNCH_POSITION);
        sleep(Consts.SLEEP_BEFORE_RESET_SERVO_POSITION);
        pushServo.setPosition(Consts.SERVO_REST_POSITION);

        if ((leftOutakeMotor.getVelocity() >= setMinVelocity) && (rightOutakeMotor.getVelocity() >= setMinVelocity))
        {
            //Step 7. position servo into launch position
            pushServo.setPosition(Consts.SERVO_LAUNCH_POSITION);
            sleep(Consts.SLEEP_BEFORE_RESET_SERVO_POSITION);
            pushServo.setPosition(Consts.SERVO_REST_POSITION);
        }
    }
    public void RunGamepadA(DcMotorEx leftOutakeMotor, DcMotorEx rightOutakeMotor, Servo pushServo) {
        setMinVelocity = Consts.MIN_VELOCITY_BACK_LAUNCH_ZONE;
        setTargetVelocity = Consts.TARGET_VELOCITY_BACK_LAUNCH_ZONE;
        leftOutakeMotor.setVelocity(setMinVelocity);
        rightOutakeMotor.setVelocity(setMinVelocity);
        leftOutakeMotor.setVelocity(setTargetVelocity);
        rightOutakeMotor.setVelocity(setTargetVelocity);
        pushServo.setPosition(Consts.SERVO_REST_POSITION);
        //telemetry.update();
        // Step 6 and Step 7 are performed upon pressing dpad_up.
    }
    public void RunDpadUp(DcMotorEx leftOutakeMotor, DcMotorEx rightOutakeMotor, Servo pushServo){
        pushServo.setPosition(Consts.SERVO_REST_POSITION);pushServo.setPosition(Consts.SERVO_LAUNCH_POSITION);
        // while (pushServo.getPosition() > Consts.SERVO_LAUNCH_POSITION) idle();
        sleep(Consts.SLEEP_BEFORE_RESET_SERVO_POSITION);
        pushServo.setPosition(Consts.SERVO_REST_POSITION);

//        if ((leftOutakeMotor.getVelocity() >= setMinVelocity) && (rightOutakeMotor.getVelocity() >= setMinVelocity))
//        {
//            //Step 7. position servo into launch position
//            pushServo.setPosition(Consts.SERVO_LAUNCH_POSITION);
//            sleep(Consts.SLEEP_BEFORE_RESET_SERVO_POSITION);
//            pushServo.setPosition(Consts.SERVO_REST_POSITION);
//        }
    }

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
            allianceColor = Consts.AllianceColor.RED;
            if (gamepad1.a) OneGamepadAControl = true;
            if (gamepad1.b) OneGamepadAControl = false;
            telemetry.addData("GamepadA Control ALL", OneGamepadAControl);
            if (gamepad1.right_bumper)
            {
                allianceColor = Consts.AllianceColor.BLUE;
                // Display the current selection on the Driver Station
                telemetry.addData("Alliance", "Selected: ", "BLUE");
            } else if (gamepad1.left_bumper) {
                allianceColor = Consts.AllianceColor.RED;
                // Display the current selection on the Driver Station
                telemetry.addData("Alliance", "Selected: ", "RED");
            } else {
                allianceColor = Consts.AllianceColor.RED;
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
         * BRAKE might damage internal motor components.
         */
        rightOutakeMotor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        leftOutakeMotor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);

        leftOutakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Consts.leftPIDF);
        rightOutakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Consts.rightPIDF);
        if (allianceColor == Consts.AllianceColor.RED)
        {
            // Starting position Red Goal
            startingPose = Consts.RED_STARTING_POSE;
            follower.setStartingPose(startingPose);
            follower.update();
            telemetry.addData("Alliance Color", "Red");
            telemetry.addData("Starting Pose", follower.getPose());
        }
        else if(allianceColor == Consts.AllianceColor.BLUE)
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
            telemetry.addData("Left Outake Motor Velocity Front:", leftOutakeMotor.getVelocity());
            telemetry.addData("Right Outake Motor Velocity Front:", rightOutakeMotor.getVelocity());
            telemetry.addData("Target Velocity front", setTargetVelocity);
            telemetry.addData("Min Velocity front", setMinVelocity);
            if (gamepad2.a || (OneGamepadAControl&&gamepad1.a)){
                RunGamepadA(leftOutakeMotor,rightOutakeMotor,pushServo);
            }

            // Step 6. Check if the velocity of the motors is more than the min velocity
            if (gamepad2.dpad_up || (OneGamepadAControl&&gamepad1.dpad_up))
            {
                LLRunDpadUp(leftOutakeMotor,rightOutakeMotor,pushServo);
            }

            if (gamepad2.b || (OneGamepadAControl&&gamepad1.b)) {
                leftOutakeMotor.setVelocity(Consts.STOP_VELOCITY);
                rightOutakeMotor.setVelocity(Consts.STOP_VELOCITY);
                pushServo.setPosition(Consts.SERVO_REST_POSITION);
            }


            intakeMotor.setPower(gamepad2.x||(OneGamepadAControl&&gamepad1.x)?1:0);

            if(gamepad2.y || (OneGamepadAControl&&gamepad1.y)) {
                setMinVelocity = Consts.MIN_VELOCITY_FRONT_LAUNCH_ZONE;
                setTargetVelocity = Consts.TARGET_VELOCITY_FRONT_LAUNCH_ZONE;

                leftOutakeMotor.setVelocity(setTargetVelocity);
                rightOutakeMotor.setVelocity(setTargetVelocity);
                pushServo.setPosition(Consts.SERVO_REST_POSITION);
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