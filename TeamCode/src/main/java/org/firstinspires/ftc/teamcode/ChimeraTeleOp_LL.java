package org.firstinspires.ftc.teamcode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.bylazar.telemetry.TelemetryManager;
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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Systems.Consts;
import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode.pedroAuto.Constants;


@TeleOp(name = "ChimeraTeleOp New", group = "AbsolutePriority")// Name and Group
public class ChimeraTeleOp_LL extends LinearOpMode {
    public LimelightSystem.ChimeraLL limelight = new LimelightSystem.ChimeraLL();
    final double TARGET_VELOCITY = 3000; // Set target velocity- in RPM(e.g., 3000 RPM)
    final double TARGET_VELOCITY_BACK_LAUNCH_ZONE = 1150;// Set target velocity from back launch zone
    final double TARGET_VELOCITY_FRONT_LAUNCH_ZONE = 1040;// Set target velocity from front launch zone
    final double MIN_VELOCITY_BACK_LAUNCH_ZONE = 1050;// Set target velocity from back launch zone
    final double MIN_VELOCITY_FRONT_LAUNCH_ZONE = 100;// Set target velocity from back launch zone
    final double STOP_VELOCITY = 0; // Set target velocity- in RPM(e.g., 3000 RPM)
    final double MIN_VELOCITY = 1075;
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    final int SERVO_LAUNCH_POSITION = 0;
    final int SERVO_REST_POSITION = 1;
    final int SLEEP_BEFORE_RESET_SERVO_POSITION = 200;

    // declaring our PIDF tuning values
    final double Kp = 300;
    final double Ki = 0.0;
    final double Kd = 0.0;
    final double Kf = 10;
    double  setTargetVelocity;
    double setMinVelocity = 0;
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive = false;
    private boolean slowMode = false;
    private TelemetryManager telemetryM;
    double X_Coordinate_Blue_Goal = 0;
    double Y_Coordinate_Blue_Goal = 144;
    double X_Coordinate_Red_Goal = 144;
    double Y_Coordinate_Red_Goal = 144;
    double X_Coordinate = 0.0;
    double Y_Coordinate = 0.0;
    double Distance_To_Goal = 0;
    double Distance_To_Goal_Blue = 0;
    double currentHeading = 0.0;
    double launchPositionHeadingRadians = 0;
    final double SHOOTING_ZONE_CLOSE_FRONT_LAUNCH_ZONE = 20;
    final double SHOOTING_ZONE_FAR_FRONT_LAUNCH_ZONE = 60;
    final double SHOOTING_ZONE_BACK_LAUNCH_ZONE = 65;

    // TODO Change Starting position. Temporarily set starting position to back launch
    // zone, (x,y) = (72,0)
    final double RED_ALLIANCE_STARTING_X_COORDINATE = 104;
    final double RED_ALLIANCE_STARTING_Y_COORDINATE = 60;
    final double RED_ALLIANCE_STARTING_HEADING_POSITION = 180;

    final double BLUE_ALLIANCE_STARTING_X_COORDINATE = 144;
    final double BLUE_ALLIANCE_STARTING_Y_COORDINATE = 0;
    final double BLUE_ALLIANCE_STARTING_HEADING_POSITION = 90;
    Consts.AllianceColor allianceColor;

    ElapsedTime feederTimer = new ElapsedTime();

    public double findAngleToRotate() {
        // return allianceColor==Consts.AllianceColor.RED? Math.atan2( (Consts.Y_Coordinate_Red_Goal - curPose.getY()), (Consts.X_Coordinate_Red_Goal - curPose.getX()) ):Math.atan2( (Consts.Y_Coordinate_Blue_Goal - curPose.getY()), (Consts.X_Coordinate_Blue_Goal - curPose.getX()) );
        return Math.toRadians(limelight.tx);
    }
    public Path getPath(double amt) {
        Pose pose = follower.getPose();
        double heading = pose.getHeading();
        double RobotCentric_X = Math.sin(heading);
        double RobotCentric_Y = Math.cos(heading);
        Pose endPose = new Pose(pose.getX() + RobotCentric_X*amt, pose.getY() + RobotCentric_Y*amt, heading);
        Path path = new Path(new BezierLine(pose, endPose));path.setLinearHeadingInterpolation(heading,heading);
        return path;
    }
    public void rotate(double angle_Radians) {follower.turn(Math.abs(angle_Radians), angle_Radians/Math.abs(angle_Radians)==-1);follower.update();}

    @Override
    public void runOpMode() throws InterruptedException {
        //while (!isStarted() && !isStopRequested())
        while (opModeInInit())
        {
            telemetry.addData("Press 'GamePad1 Right Bumper'", "for BLUE");
            telemetry.addData("Press 'GamePad1 Left Bumper'", "for RED");
            // This method is called repeatedly during the init phase
            allianceColor = Consts.AllianceColor.RED;
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
        rightOutakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftOutakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
        rightOutakeMotor.setZeroPowerBehavior(BRAKE);
        leftOutakeMotor.setZeroPowerBehavior(BRAKE);

        leftOutakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Kp, Ki, Kd, Kf));
        rightOutakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Kp, Ki, Kd, Kf));
        if (allianceColor == Consts.AllianceColor.RED)
        {
            // Starting position Red Goal
            startingPose = new Pose(RED_ALLIANCE_STARTING_X_COORDINATE, RED_ALLIANCE_STARTING_Y_COORDINATE, Math.toRadians(RED_ALLIANCE_STARTING_HEADING_POSITION));
            follower.setStartingPose(startingPose);
            follower.update();
            telemetry.addData("Alliance Color", "Red");
            telemetry.addData("Starting Pose", follower.getPose());
        }
        else if(allianceColor == Consts.AllianceColor.BLUE)
        {
            startingPose = new Pose(BLUE_ALLIANCE_STARTING_X_COORDINATE, BLUE_ALLIANCE_STARTING_Y_COORDINATE, Math.toRadians(BLUE_ALLIANCE_STARTING_HEADING_POSITION));
            follower.setStartingPose(startingPose);
            follower.update();
            telemetry.addData("Alliance Color", "Blue");
            telemetry.addData("Starting Pose", follower.getPose());
        } else {
            // Starting position Red Goal
            startingPose = new Pose(RED_ALLIANCE_STARTING_X_COORDINATE, RED_ALLIANCE_STARTING_Y_COORDINATE, Math.toRadians(RED_ALLIANCE_STARTING_HEADING_POSITION));
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

        telemetry.addData("Status", "Initialized");
        limelight.setDevice(hardwareMap.get(Limelight3A.class, "limelight"));
        waitForStart();
        limelight.startLLWithPipeline(allianceColor==Consts.AllianceColor.RED?4:5);
        follower.startTeleopDrive();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            limelight.LLUpdate();
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
            //Step 1. Find the position of the robot on the field
            X_Coordinate = follower.getPose().getX();
            Y_Coordinate = follower.getPose().getY();
            currentHeading =  follower.getPose().getHeading();

            if (allianceColor == AllianceColor.RED)
            {
                telemetry.addData("Alliance Color", "Red");
            }
            else if(allianceColor == AllianceColor.BLUE)
            {
                telemetry.addData("Alliance Color", "Blue");
            }

            telemetry.addData("Current X Coordinate", X_Coordinate);
            telemetry.addData("Current Y Coordinate", Y_Coordinate);
            telemetry.addData("Current heading", Math.toDegrees(currentHeading));
//            telemetry.addData("X Coordinate Red Goal", X_Coordinate_Red_Goal);
//            telemetry.addData("Y Coordinate Red Goal", Y_Coordinate_Red_Goal);
//            telemetry.addData("X Coordinate Blue Goal", X_Coordinate_Blue_Goal);
//            telemetry.addData("Y Coordinate Blue Goal", Y_Coordinate_Blue_Goal);
            */
            /*
             * ------- gamepad2.a:
             * Step 1. Find the heading and rotate to face the goal
             * Step 2. Move forward/backward so that the robot is the correct distance from the goal
             * Step 3. Start the outtake motors
             * ------- dpad_up:
             * Step 4. Check if the velocity of the motors is more than the min velocity
             * Step 5. position servo into launch position
             */


            if (gamepad2.a) {
                // STEP 1
                rotate(findAngleToRotate() * Consts.LAUNCHER_GOALTAG_ANGLE_SCALE);
                // STEP 2
                follower.followPath(getPath(limelight.dist - Consts.LAUNCHER_GOALTAG_OFFSET));follower.update();
                // STEP 3
                setMinVelocity = MIN_VELOCITY_BACK_LAUNCH_ZONE;setTargetVelocity = TARGET_VELOCITY_BACK_LAUNCH_ZONE;
                leftOutakeMotor.setVelocity(setTargetVelocity);rightOutakeMotor.setVelocity(setTargetVelocity);
                pushServo.setPosition(SERVO_REST_POSITION);
            }
            if (gamepad2.dpad_up)
            {
                telemetry.addData("Launch: Left Outake Motor Velocity", leftOutakeMotor.getVelocity());
                telemetry.addData("Launch: Right Outake Motor Velocity", rightOutakeMotor.getVelocity());
                telemetry.addData("Launch: Min Velocity ", setMinVelocity);
                pushServo.setPosition(SERVO_LAUNCH_POSITION);
                sleep(SLEEP_BEFORE_RESET_SERVO_POSITION);
                pushServo.setPosition(SERVO_REST_POSITION);

                if ((leftOutakeMotor.getVelocity() >= setMinVelocity) && (rightOutakeMotor.getVelocity() >= setMinVelocity))
                {
                    //Step 7. position servo into launch position
                    pushServo.setPosition(SERVO_LAUNCH_POSITION);
                    sleep(SLEEP_BEFORE_RESET_SERVO_POSITION);
                    pushServo.setPosition(SERVO_REST_POSITION);
                }


            }

            if (gamepad2.b) {
                leftOutakeMotor.setVelocity(STOP_VELOCITY);
                rightOutakeMotor.setVelocity(STOP_VELOCITY);
                pushServo.setPosition(SERVO_REST_POSITION);
            }


            intakeMotor.setPower(gamepad2.x?1:0);


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