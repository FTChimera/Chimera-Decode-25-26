package org.firstinspires.ftc.teamcode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Systems.Consts;
import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode.Systems.RGBIndicator;
import org.firstinspires.ftc.teamcode.pedroAuto.Constants;

@TeleOp(name = "ChimeraTeleOp", group = "AbsolutePriority")
public class ChimeraTeleOp extends LinearOpMode {
    public LimelightSystem.ChimeraLL limelight = new LimelightSystem.ChimeraLL();
    double setTargetVelocity;
    double setMinVelocity = 0;
    private Follower follower;
   // private MultipleTelemetry telemetry;

    public static Pose startingPose;

    Consts.AllianceColor allianceColor;
    RGBIndicator rgbIndicator;
    boolean OneGamepadAControl;

    // --- NEW: Non-Blocking Launch Variables ---
    private ElapsedTime launchTimer = new ElapsedTime();
    private boolean isLaunching = false;
    private boolean dpadWasPressed = false;

    // --- LIMELIGHT & PATHING METHODS ---
    public double findAngleToRotate() {
        return Math.toRadians(limelight.tx);
    }

    public Path getPath(double amt) {
        Pose pose = follower.getPose();
        double heading = pose.getHeading() - (allianceColor == Consts.AllianceColor.RED ? 0.79 : -0.79);
        double RobotCentric_X = Math.cos(heading);
        double RobotCentric_Y = Math.sin(heading);
        Pose endPose = new Pose(pose.getX() + RobotCentric_X * amt, pose.getY() + RobotCentric_Y * amt, heading);
        Path path = new Path(new BezierLine(pose, endPose));
        path.setLinearHeadingInterpolation(heading, heading);
        return path;
    }

    public void rotate(double angle_Radians) {
        follower.turn(Math.abs(angle_Radians), angle_Radians / Math.abs(angle_Radians) == -1);
        follower.update();
    }

    // --- ORIGINAL AUTO-ALIGN SEQUENCE ---
    public void LLRunGamepadA(DcMotorEx leftOutakeMotor, DcMotorEx rightOutakeMotor, Servo pushServo) {
        // STEP 1
        rotate(findAngleToRotate() * Consts.LAUNCHER_GOALTAG_ANGLE_SCALE);
        // STEP 2
        follower.followPath(getPath(limelight.dist));
        follower.update();
        // STEP 3
        setMinVelocity = Consts.MIN_VELOCITY_BACK_LAUNCH_ZONE;
        setTargetVelocity = Consts.TARGET_VELOCITY_BACK_LAUNCH_ZONE;
        leftOutakeMotor.setVelocity(setTargetVelocity);
        rightOutakeMotor.setVelocity(setTargetVelocity);
        pushServo.setPosition(Consts.SERVO_REST_POSITION);
    }

    // --- NEW: NON-BLOCKING LAUNCH SEQUENCE (Replaces LLRunDpadUp) ---
    public void handleLaunchSequence(DcMotorEx leftOutakeMotor, DcMotorEx rightOutakeMotor, Servo pushServo) {
        boolean dpadIsPressed = gamepad2.dpad_up || (OneGamepadAControl && gamepad1.dpad_up);

        // Rising Edge Detection: Only trigger if button is newly pressed AND we aren't already launching
        if (dpadIsPressed && !dpadWasPressed && !isLaunching) {
            // Velocity Check
            if (leftOutakeMotor.getVelocity() >= setMinVelocity && rightOutakeMotor.getVelocity() >= setMinVelocity) {
                // Fire!
                pushServo.setPosition(Consts.SERVO_LAUNCH_POSITION);
                launchTimer.reset();
                isLaunching = true;
            }
        }

        // Update previous button state for the next loop
        dpadWasPressed = dpadIsPressed;

        // Timer Logic: Retract servo after time passes
        if (isLaunching && launchTimer.milliseconds() >= Consts.SLEEP_BEFORE_RESET_SERVO_POSITION) {
            pushServo.setPosition(Consts.SERVO_REST_POSITION);
            isLaunching = false;
        }

        // Telemetry for debugging
        telemetry.addData("Launch Status", isLaunching ? "FIRING" : "READY");
        telemetry.addData("Launch Timer", launchTimer.milliseconds());
    }

    // --- ORIGINAL MANUAL SPIN UP ---
    public void RunGamepadA(DcMotorEx leftOutakeMotor, DcMotorEx rightOutakeMotor, Servo pushServo) {
        setMinVelocity = Consts.MIN_VELOCITY_BACK_LAUNCH_ZONE;
        setTargetVelocity = Consts.TARGET_VELOCITY_BACK_LAUNCH_ZONE;
        leftOutakeMotor.setVelocity(setMinVelocity);
        rightOutakeMotor.setVelocity(setMinVelocity);
        leftOutakeMotor.setVelocity(setTargetVelocity);
        rightOutakeMotor.setVelocity(setTargetVelocity);
        pushServo.setPosition(Consts.SERVO_REST_POSITION);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        rgbIndicator = new RGBIndicator(hardwareMap.get(Servo.class, "rgb"));
        rgbIndicator.setColor(RGBIndicator.Color.VIOLET);
       // telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        // --- INIT LOOP ---
        while (opModeInInit()) {
            telemetry.addData("Press 'GamePad1 Right Bumper'", "for BLUE");
            telemetry.addData("Press 'GamePad1 Left Bumper'", "for RED");

            if (gamepad1.aWasPressed()) OneGamepadAControl = !OneGamepadAControl;
            telemetry.addData("GamepadA Control ALL (press A to switch)", OneGamepadAControl);

            if (gamepad1.right_bumper) {
                allianceColor = Consts.AllianceColor.BLUE;
                telemetry.addData("Alliance", "Selected: ", "BLUE");
            } else if (gamepad1.left_bumper) {
                allianceColor = Consts.AllianceColor.RED;
                telemetry.addData("Alliance", "Selected: ", "RED");
            } else {
                allianceColor = Consts.AllianceColor.RED;
                telemetry.addData("Alliance", "Selected: ", "RED");
            }
            telemetry.update();
        }

        // --- HARDWARE MAPPING ---
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        DcMotorEx rightOutakeMotor = hardwareMap.get(DcMotorEx.class, "OutakeMotorRight");
        DcMotorEx leftOutakeMotor = hardwareMap.get(DcMotorEx.class, "OutakeMotorLeft");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        Servo pushServo = hardwareMap.servo.get("pushServo");

        follower = Constants.createFollower(hardwareMap);

        // --- MOTOR CONFIGURATION ---
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightOutakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftOutakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftOutakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOutakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightOutakeMotor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        leftOutakeMotor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);

        leftOutakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Consts.leftPIDF);
        rightOutakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Consts.rightPIDF);

        // --- POSE SETUP ---
        if (allianceColor == Consts.AllianceColor.RED) {
            startingPose = Consts.TELEOP_RED_STARTING_POSE;
        } else if (allianceColor == Consts.AllianceColor.BLUE) {
            startingPose = Consts.TELEOP_BLUE_STARTING_POSE;
        } else {
            startingPose = Consts.TELEOP_RED_STARTING_POSE;
        }

        follower.setStartingPose(startingPose);
        follower.update();
        telemetry.addData("Alliance Color", allianceColor);
        telemetry.addData("Starting Pose", follower.getPose());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        limelight.setDevice(hardwareMap.get(Limelight3A.class, "limelight"));

        waitForStart();

        limelight.startLLWithPipeline(0);
        follower.startTeleopDrive();

        if (isStopRequested()) return;
        telemetry.addData("Status", "Running");

        // --- MAIN LOOP ---
        while (opModeIsActive()) {
            // 1. Update Sensors
            limelight.LLUpdate();

            // 2. LED Indicators
            telemetry.addData("Angle Degrees", limelight.getLLScore());
            if (limelight.getLLScore() == 0) rgbIndicator.setColor(RGBIndicator.Color.BLACK);
            else if (limelight.getLLScore() < 1.5) rgbIndicator.setColor(RGBIndicator.Color.GREEN);
            else if (limelight.getLLScore() < 5) rgbIndicator.setColor(RGBIndicator.Color.YELLOW);
            else if (limelight.getLLScore() < 12.5) rgbIndicator.setColor(RGBIndicator.Color.ORANGE);
            else rgbIndicator.setColor(RGBIndicator.Color.BLACK);

            // 3. Drivetrain Control
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            follower.update();

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = 1 * (y + x + rx) / denominator;
            double backLeftPower = 1 * (y - x + rx) / denominator;
            double frontRightPower = 1 * (y - x - rx) / denominator;
            double backRightPower = 1 * (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // 4. Launcher Logic
            telemetry.addData("Left Outake Velocity", leftOutakeMotor.getVelocity());
            telemetry.addData("Right Outake Velocity", rightOutakeMotor.getVelocity());
            telemetry.addData("Target Velocity", setTargetVelocity);
            telemetry.addData("Min Velocity", setMinVelocity);

            // Manual Spin-up (Gamepad A)
            if (gamepad2.a || (OneGamepadAControl && gamepad1.a)) {
                // If using Limelight auto-align feature:
                // LLRunGamepadA(leftOutakeMotor, rightOutakeMotor, pushServo);

                // If using Manual Spin up (as per original code flow for button A):
                RunGamepadA(leftOutakeMotor, rightOutakeMotor, pushServo);
            }

            // Fire Sequence (Replaces old LLRunDpadUp logic)
            // This handles the Servo push AND the timer reset
            handleLaunchSequence(leftOutakeMotor, rightOutakeMotor, pushServo);

            // Stop Mechanisms (Gamepad B)
            if (gamepad2.b || (OneGamepadAControl && gamepad1.b)) {
                leftOutakeMotor.setVelocity(Consts.STOP_VELOCITY);
                rightOutakeMotor.setVelocity(Consts.STOP_VELOCITY);
                pushServo.setPosition(Consts.SERVO_REST_POSITION);
                isLaunching = false; // Reset launch state if manual stop is pressed
            }

            // Custom Launch Velocity (Use Bumper of +- 50 RPM)
            if (gamepad2.rightBumperWasPressed() || (OneGamepadAControl && gamepad1.rightBumperWasPressed())) {
                setTargetVelocity += 25;
                leftOutakeMotor.setVelocity(setTargetVelocity);
                rightOutakeMotor.setVelocity(setTargetVelocity);
            }
            if (gamepad2.leftBumperWasPressed() || (OneGamepadAControl && gamepad1.leftBumperWasPressed())) {
                setTargetVelocity -= 25;
                leftOutakeMotor.setVelocity(setTargetVelocity);
                rightOutakeMotor.setVelocity(setTargetVelocity);
            }

            // Intake (Gamepad X)
            intakeMotor.setPower(gamepad2.x || (OneGamepadAControl && gamepad1.x) ? 1 : 0);


            if (gamepad2.y || (OneGamepadAControl && gamepad1.y)) {
                setMinVelocity = Consts.MIN_VELOCITY_FRONT_LAUNCH_ZONE;
                setTargetVelocity = Consts.TARGET_VELOCITY_FRONT_LAUNCH_ZONE;
                leftOutakeMotor.setVelocity(setTargetVelocity);
                rightOutakeMotor.setVelocity(setTargetVelocity);
                pushServo.setPosition(Consts.SERVO_REST_POSITION);
            }

            telemetry.update();
        }
    }
}
