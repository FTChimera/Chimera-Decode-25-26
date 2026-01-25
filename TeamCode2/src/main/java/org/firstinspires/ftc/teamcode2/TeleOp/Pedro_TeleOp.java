package org.firstinspires.ftc.teamcode2.TeleOp;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode2.Auto.BLUE_AUTO;
import org.firstinspires.ftc.teamcode2.Auto.RED_AUTO;
import org.firstinspires.ftc.teamcode2.Systems.Consts;
import org.firstinspires.ftc.teamcode2.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode2.Systems.PIDFController;
import org.firstinspires.ftc.teamcode2.Systems.RGBIndicator;
import org.firstinspires.ftc.teamcode2.pedroPathing.Constants;


@Configurable
@TeleOp(name = "Pedro_TeleOp", group = "TeleOp")
public class Pedro_TeleOp extends OpMode {
    /*
    * Note that all variables here are updated.
    * Make sure that every constant final variable is kept in Consts.java
    * FIGURE OUT HOW TO COMPLETE TELEOP
    */
    private LimelightSystem limelight;
    private RGBIndicator rgbIndicator;
    private Follower follower;
    private Consts.AllianceColor allianceColor = Consts.AllianceColor.RED;
    private PIDFController ll_PIDF = new PIDFController(
            Consts.LimelightAutoAlignmentTurning,
            -1,1,10
    );
    private boolean automatedDrive=false, launcherOn=false;
    private long lastTimeNs,nowNs;
    double dt;
    private enum LaunchingState {
        IDLE,
        GOING_UP,
        LAUNCHING,
        GOING_DOWN
    } private LaunchingState launchingState = LaunchingState.IDLE;
    protected TelemetryManager telemetryM;
    public Pose startingPose;
    DcMotorEx launcher; DcMotor intake; Servo pushServo;
    Timer Servo_timer;

    @Override
    public void init() {
        limelight = new LimelightSystem(hardwareMap);
        rgbIndicator = new RGBIndicator(hardwareMap.get(Servo.class, "rgb"));
        pushServo = hardwareMap.servo.get("push");
        Servo_timer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.dcMotor.get("intake");

        launcher.setZeroPowerBehavior(FLOAT);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Consts.LaunchPIDF);
        pushServo.setPosition(Consts.SERVO_DOWN_POSITION);
    }
    @Override
    public void init_loop() {
        telemetryM.addData("Alliance Color (press A to switch)", allianceColor);
        if (gamepad1.aWasPressed()) {
            allianceColor = allianceColor.switchColors();
        }
        // Set starting pose based on alliance color
        if (allianceColor == Consts.AllianceColor.RED) {
            startingPose = RED_AUTO.endPose; // Auto end pose is TeleOp start pose
        } else {
            startingPose = BLUE_AUTO.endPose;
        }

        telemetryM.addData("Starting Pose", startingPose);
        telemetryM.update();
    }

    @Override
    public void start() {
        // Limelight start with pipeline 0 for april tags
        limelight.start(0);
        // Follower stuff
        follower.setStartingPose(startingPose);
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constants.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        limelight.LLUpdate();
        rgbIndicator.updateUsingLL(limelight);
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //This is the normal version to use in the TeleOp
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y* Consts.DRIVE_SCALAR,
                    -gamepad1.left_stick_x* Consts.DRIVE_SCALAR,
                    -gamepad1.right_stick_x* Consts.DRIVE_SCALAR,
                    true // Robot Centric
            );
        }

        //Automated Path Following
        if (gamepad1.aWasPressed()) {
            // Find which launch zone is closer
            Pose backLaunchZone = allianceColor == Consts.AllianceColor.RED ?
                    Consts.RED_SHOOTING_BACK : Consts.BLUE_SHOOTING_BACK;
            Pose frontLaunchZone = allianceColor == Consts.AllianceColor.RED ?
                    Consts.RED_SHOOTING_FRONT : Consts.BLUE_SHOOTING_FRONT;
            boolean shootFromBack = IsBackLaunchZoneCloser();
            // Follow path to the closer launch zone
            Path pathToLaunchZone = new Path(
                    new BezierLine(follower.getPose(), shootFromBack ? backLaunchZone : frontLaunchZone)
            );
            pathToLaunchZone.setLinearHeadingInterpolation(
                    follower.getHeading(),
                    (shootFromBack ? backLaunchZone.getHeading() : frontLaunchZone.getHeading())
            );
            automatedDrive = true;
        }

        // Parking Aid
        if (gamepad1.yWasPressed()) {
            // To be fully inside the parking zone, the heading should only be either 0,90,180,270
            // First, find which heading is closer.
            double heading = Math.toDegrees(follower.getHeading());
            heading = heading / 90; // make values we want integers and the rest are floating-point numbers
            heading = Math.round(heading); // round to nearest integer
            heading = heading * 90; // go back to before
            heading = heading % 360; // Apply modulo operator to convert 360 to 0.
            // Follow path to the parking zone
            Pose parkingPose = allianceColor == Consts.AllianceColor.RED ?
                    Consts.RED_PARKING : Consts.BLUE_PARKING;
            Path pathToParking = new Path(
                    new BezierLine(follower.getPose(), parkingPose)
            );
            pathToParking.setLinearHeadingInterpolation(
                    follower.getHeading(),
                    heading
            );
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
        // Auto Alignment using Limelight
        nowNs = System.nanoTime();
        dt = (nowNs - lastTimeNs) / 1e9;
        lastTimeNs = nowNs;

        // Manual-stick override: if driver is giving significant rotation input, reset PID
        if (Math.abs(gamepad1.right_stick_x) > 0.12) {
            ll_PIDF.reset();
        }
        if (gamepad1.x) {
            double error = limelight.tx;
            double rotationCmd = ll_PIDF.update(error, dt);
            follower.setTeleOpDrive(
                    0,
                    0,
                    rotationCmd, // Proportional control for turning
                    true
            );
        } else {
            // Not auto-aiming, ensure PID state cleared so integral doesn't accumulate unseen error
            // (optional: only reset on button release if you want integral to persist)
            ll_PIDF.reset();
        }
        // Launcher and Intake
        if (gamepad1.leftBumperWasPressed()) {
            // Run launcher
            if (!launcherOn) {
                // Set launcher velocity based on launch zone
                if (IsBackLaunchZoneCloser()) {
                    launcher.setVelocity(Consts.MIN_VELOCITY_BACK_LAUNCH_ZONE);
                    launcher.setVelocity(Consts.TARGET_VELOCITY_BACK_LAUNCH_ZONE);
                } else {
                    launcher.setVelocity(Consts.MIN_VELOCITY_FRONT_LAUNCH_ZONE);
                    launcher.setVelocity(Consts.TARGET_VELOCITY_FRONT_LAUNCH_ZONE);
                }
                launcherOn = true;
            } else {
                launcher.setVelocity(Consts.STOP_VELOCITY);
                launcherOn = false;
            }
        }
        // ALL SERVO LAUNCHING LOGIC
        if (gamepad1.right_bumper && launchingState==LaunchingState.IDLE) {
            // In IDLE, we don't do anything
            launchingState = LaunchingState.GOING_UP;
            Servo_timer.resetTimer();
        }

        if (launchingState==LaunchingState.GOING_UP) {
            // This is the Fire logic.
            // Wait until the launcher reaches past the Min velocity
            if (launcher.getVelocity() >= Consts.MIN_VELOCITY_BACK_LAUNCH_ZONE && IsBackLaunchZoneCloser()
            || launcher.getVelocity() >= Consts.MIN_VELOCITY_FRONT_LAUNCH_ZONE && !IsBackLaunchZoneCloser()) {
                pushServo.setPosition(Consts.SERVO_UP_POSITION);
                launchingState = LaunchingState.LAUNCHING;
                Servo_timer.resetTimer();
            }

        }
        if (launchingState==LaunchingState.LAUNCHING) {
            if (Servo_timer.getElapsedTimeSeconds()/1000 >= Consts.SLEEP_BEFORE_RESET_SERVO_POSITION) {
                launchingState = LaunchingState.GOING_DOWN;
                Servo_timer.resetTimer();
            }
        }

        if (launchingState==LaunchingState.GOING_DOWN) {
            pushServo.setPosition(Consts.SERVO_DOWN_POSITION);
            launchingState = LaunchingState.IDLE;
            Servo_timer.resetTimer();
        }
        // INCREASE/DECREASE LAUNCHER VELOCITY
        if (gamepad1.dpadUpWasPressed()) {
            launcher.setVelocity(launcher.getVelocity()+25);
        } else if (gamepad1.dpadDownWasPressed()) {
            launcher.setVelocity(launcher.getVelocity()-25);
        }
        // INTAKE CONTROL
        intake.setPower(Math.max(Math.min(gamepad1.left_trigger - gamepad1.right_trigger *1.1,1),-1)); //Counteract imperfect intake power
        telemetryM.debug("Angle (in degrees)", limelight.tx); // LLScore is negative/positive
        telemetryM.debug("Launcher Velocity", launcher.getVelocity());
        telemetryM.debug("Servo data" + launchingState);
        if (launchingState != LaunchingState.IDLE) telemetryM.debug("Servo Timer (ms)", Servo_timer.getElapsedTimeSeconds()/1000);
    }
    public boolean IsBackLaunchZoneCloser() {
        Pose backLaunchZone = allianceColor == Consts.AllianceColor.RED ?
                Consts.RED_SHOOTING_BACK : Consts.BLUE_SHOOTING_BACK;
        Pose frontLaunchZone = allianceColor == Consts.AllianceColor.RED ?
                Consts.RED_SHOOTING_FRONT : Consts.BLUE_SHOOTING_FRONT;
        return getDistanceFromTwoPosesPP(follower.getPose(), backLaunchZone) <
                getDistanceFromTwoPosesPP(follower.getPose(), frontLaunchZone);
    }

    public double getDistanceFromTwoPosesPP(Pose pose1, Pose pose2) {return Math.hypot(pose2.getX() - pose1.getX(), pose2.getY() - pose1.getY());}
}