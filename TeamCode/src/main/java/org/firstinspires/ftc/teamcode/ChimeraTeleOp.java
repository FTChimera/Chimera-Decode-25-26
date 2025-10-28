package org.firstinspires.ftc.teamcode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static java.lang.Math.sqrt;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroAuto.Constants;


@TeleOp(name = "ChimeraTeleOp", group = "AbsolutePriority")// Name and Group
public class ChimeraTeleOp extends LinearOpMode {

    final double TARGET_VELOCITY = 3000; // Set target velocity- in RPM(e.g., 3000 RPM)
    final double TARGET_VELOCITY_BACK_LAUNCH_ZONE = 1200;// Set target velocity from back launch zone
    final double TARGET_VELOCITY_FRONT_LAUNCH_ZONE = 1000;// Set target velocity from back launch zone
    final double MIN_VELOCITY_BACK_LAUNCH_ZONE = 500;// Set target velocity from back launch zone
    final double MIN_VELOCITY_FRONT_LAUNCH_ZONE = 100;// Set target velocity from back launch zone
    final double STOP_VELOCITY = 0; // Set target velocity- in RPM(e.g., 3000 RPM)
    final double MIN_VELOCITY = 1075;
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    final int SERVO_LAUNCH_POSITION = 0;
    final int SERVO_REST_POSITION = 1;
    final int SLEEP_BEFORE_RESET_SERVO_POSITION = 500;
    // declaring our PIDF tuning values
    final double Kp = 300;

    final double Ki = 0.0;
    final double Kd = 0.0;
    final double Kf = 10;
    double  setTargetVelocity = 0;
    double setMinVelocity = 0;
    private Follower follower;
    double X_Coordinate_Blue_Goal = 0;
    double Y_Coordinate_Blue_Goal = 144;
    double X_Coordinate_Red_Goal = 144;
    double Y_Coordinate_Red_Goal = 144;
    double X_Coordinate = 0.0;
    double Y_Coordinate = 0.0;
    double heading = 0.0;
    double Distance_To_Goal_Red = 0;
    double Distance_To_Goal_Blue = 0;
    double launchPostionHeading = 0;
    final double SHOOTING_ZONE_CLOSE_FRONT_LAUNCH_ZONE = 18;
    final double SHOOTING_ZONE_FAR_FRONT_LAUNCH_ZONE = 24;
    final double SHOOTING_ZONE_BACK_LAUNCH_ZONE = 36;

    ElapsedTime feederTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

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


        //  rightViperSlide = hardwareMap.dcMotor.get("rightViperSlide");
        //leftViperSlide = hardwareMap.dcMotor.get("leftViperSlide");

        //  leftViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftViperSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightViperSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
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


        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = 0.85 *(y + x + rx) / denominator;
            double backLeftPower = 0.85 *(y - x + rx) / denominator;
            double frontRightPower = 0.85 *(y - x - rx) / denominator;
            double backRightPower = 0.85* (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            /*
             * When driver presses the button to launch, 7 things need to happen
             * Step 1. Find the position of the robot on the field
             * Step 2. Determine the TARGET_VELOCITY based on robot position
             * Step 3. Determine the current heading of the robot in the field
             * Step 4. Change direction of the robot so its aimed at the goal
             * Step 5. Set TARGET_VELOCITY determined in Step 2 to start the left and right outake motors
             * Step 6. Check if the velocity of the motors is more than the min velocity
             * Step 7. position servo into launch position
             */





            if (gamepad2.a) {
                //Step 1. Find the position of the robot on the field
                // TODO- implement this-Shlok/Atul
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading", follower.getPose().getHeading());
                telemetry.update();
                X_Coordinate = follower.getPose().getX();
                Y_Coordinate = follower.getPose().getY();

                // TODO - Add code to account for if you are on the right field.



                // Step 2. Determine the TARGET_VELOCITY based on robot position
                // TODO- implement this-Shlok/Atul
                // Calculate distance from robot to the goal
                Distance_To_Goal_Red = sqrt( Math.pow((X_Coordinate_Red_Goal - X_Coordinate),2) + Math.pow((Y_Coordinate_Red_Goal - Y_Coordinate),2) );
                if (Distance_To_Goal_Red <= SHOOTING_ZONE_CLOSE_FRONT_LAUNCH_ZONE)
                {
                   // setMinVelocity = MIN_VELOCITY_FRONT_LAUNCH_ZONE;
                    //setTargetVelocity = TARGET_VELOCITY_FRONT_LAUNCH_ZONE;
                }
                else if ((Distance_To_Goal_Red > SHOOTING_ZONE_CLOSE_FRONT_LAUNCH_ZONE) &&
                        ( Distance_To_Goal_Red <= SHOOTING_ZONE_FAR_FRONT_LAUNCH_ZONE))
                {
                    // TODO: Change these values
                    //setMinVelocity = MIN_VELOCITY_FRONT_LAUNCH_ZONE;
                    //setTargetVelocity = TARGET_VELOCITY_FRONT_LAUNCH_ZONE;

                }
                else if(Distance_To_Goal_Red > SHOOTING_ZONE_FAR_FRONT_LAUNCH_ZONE)
                {
                    //setMinVelocity = MIN_VELOCITY_BACK_LAUNCH_ZONE;
                    //setTargetVelocity = TARGET_VELOCITY_BACK_LAUNCH_ZONE;

                }

                setMinVelocity = MIN_VELOCITY_BACK_LAUNCH_ZONE;
                setTargetVelocity = TARGET_VELOCITY_BACK_LAUNCH_ZONE;



                //  Step 3. Determine the current heading of the robot in the field
                launchPostionHeading = Math.atan2((Y_Coordinate_Red_Goal - Y_Coordinate), (X_Coordinate_Red_Goal - X_Coordinate) );

                // Step 4. Change direction of the robot so its aimed at the goal
                Pose scorePose = new Pose(X_Coordinate, Y_Coordinate, Math.toRadians(launchPostionHeading)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
                follower.setPose(scorePose);



                leftOutakeMotor.setVelocity(setTargetVelocity);
                rightOutakeMotor.setVelocity(setTargetVelocity);
                pushServo.setPosition(SERVO_REST_POSITION);
                telemetry.addData("Left Outake Motor Velocity Back", leftOutakeMotor.getVelocity());
                telemetry.addData("Right Outake Motor Velocity Back", rightOutakeMotor.getVelocity());
                telemetry.addData("Target Velocity Back launch Zone", setTargetVelocity);
                telemetry.addData("Min Velocity Back", setMinVelocity);

                telemetry.update();
                // Step 6. Check if the velocity of the motors is more than the min velocity
//                if ((leftOutakeMotor.getVelocity() >= setMinVelocity) && (rightOutakeMotor.getVelocity() >= setMinVelocity))  {
                //Step 7. position servo into launch position
//                    if (gamepad2.dpad_up) {
//                        pushServo.setPosition(SERVO_LAUNCH_POSITION);
//                        sleep(SLEEP_BEFORE_RESET_SERVO_POSITION);
//                        pushServo.setPosition(SERVO_REST_POSITION);
//                    }
//                }
            }

            if(gamepad2.y){
                setMinVelocity = MIN_VELOCITY_FRONT_LAUNCH_ZONE;
                setTargetVelocity = TARGET_VELOCITY_FRONT_LAUNCH_ZONE;

                leftOutakeMotor.setVelocity(setTargetVelocity);
                rightOutakeMotor.setVelocity(setTargetVelocity);
                pushServo.setPosition(SERVO_REST_POSITION);
                telemetry.addData("Left Outake Motor Velocity front", leftOutakeMotor.getVelocity());
                telemetry.addData("Right Outake Motor Velocity front", rightOutakeMotor.getVelocity());
                telemetry.addData("Target Velocity front launch Zone", setTargetVelocity);
                telemetry.addData("Min Velocity front", setMinVelocity);
                
                
            }

            // Step 6. Check if the velocity of the motors is more than the min velocity
            if (gamepad2.dpad_up)
            {
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


            while(gamepad2.left_bumper) {
                intakeMotor.setPower(1);

            }
            while(!gamepad2.left_bumper){
                intakeMotor.setPower(0);

            }

            while(gamepad2.right_bumper) {
                intakeMotor.setPower(-1);

            }
            while(!gamepad2.right_bumper){
                intakeMotor.setPower(0);

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