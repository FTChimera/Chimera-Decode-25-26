package org.firstinspires.ftc.teamcode.NewBot;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import static org.firstinspires.ftc.teamcode.NewBot.Constants.AllianceColor.RED;
import static org.firstinspires.ftc.teamcode.NewBot.Constants.STOP_VELOCITY;
import static org.firstinspires.ftc.teamcode.NewBot.Constants.TRANSFER_UP_POSITION;
import static org.firstinspires.ftc.teamcode.NewBot.Constants.VELOCITY_TOLERANCE;
import static org.firstinspires.ftc.teamcode.NewBot.Constants.applyPolynomialToDriveInputs;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode.Systems.RGBIndicator;

import java.util.List;

@SuppressWarnings("SpellCheckingInspection")
@Configurable
@TeleOp(name = "Pedro_TeleOp", group = "TeleOp")
public class TestPedroTeleOp extends OpMode {
    /*
     * Note that all variables here are updated.
     * Make sure that every constant final variable is kept in Constants.java
     * FIGURE OUT HOW TO COMPLETE TELEOP
     */

    /*
     * ------ DRIVER CONTROLS ------
     * Right trigger - intake
     * Left trigger - reverse intake
     * Left bumper - run/stop launcher
     * Right bumper - run transfer and intake to launch ball
     * A - automated drive to launch zone (which ever is closest)
     * Y - Pedro pathing automated parking aid
     * B - Stop automated drive
     * X - hold for auto alignment
     * Dpad-Up - reset IMU (do this when you are facing up)
     * Dpad-Down - reset PP Robot Pose to goal start pose
     * Left Stick - Drive
     * Right Stick - turn
     * Left Stick Button - reduce velocity by 25
     * Right Stick Button - increase velocity by 25
     * ---------------------------------------------------
     */

    // set up bulk reading on sensors
    private List<LynxModule> allHubs;
    private LimelightSystem limelight; private AutoAlignSystem autoAlignSystem;
    private RGBIndicator rgbIndicator;
    private Follower follower;
    private Constants.AllianceColor allianceColor = RED;
    // Removed redundant PID controller - now handled by AutoAlignSystem
    private boolean automatedDrive=false, launcherOn=false, robotCentric=true;
    private long lastTimeNs;
    double dt;
    private enum LaunchingState {
        IDLE,
        LAUNCHER_SPINNING_UP,
        FIRING
    }
    private LaunchingState launchingState = LaunchingState.IDLE;
    protected MultipleTelemetry telemetryM;
    private TelemetryManager panelsTelemetry;
    public Pose startingPose;
    DcMotorEx launcher; DcMotor intake, transfer;
    Timer transfer_timer;

    @Override
    public void init() {
        // set up bulk reading
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        limelight = new LimelightSystem(hardwareMap);
        rgbIndicator = new RGBIndicator(hardwareMap);
        transfer = hardwareMap.get(DcMotor.class, "transferMotor");
        transfer_timer = new Timer();
        follower = Constants.createPedroFollower(hardwareMap);
        follower.update();
        telemetryM = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        ); // cannot combine panels telemetry with normal so need to send update from panels
        // configure to update FTC Dashboard and normal at the same time
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        launcher = hardwareMap.get(DcMotorEx.class, "OuttakeMotor");
        intake = hardwareMap.dcMotor.get("intakeMotor");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setZeroPowerBehavior(FLOAT);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.LaunchPIDF);
        transfer.setPower(Constants.TRANSFER_DOWN_POSITION);
    }
    @Override
    public void init_loop() {
        panelsTelemetry.addData("Alliance Color (press A to switch)", allianceColor);
        panelsTelemetry.addData("(press B to switch) Robot Centric is", robotCentric);
        if (gamepad1.aWasPressed()) {
            allianceColor = allianceColor.switchColors();
        }
        if (gamepad1.bWasPressed()) {
            robotCentric = !robotCentric;
        }
        // Set starting pose based on alliance color
        if (allianceColor == RED) {}

        panelsTelemetry.addData("Starting Pose", startingPose);
        panelsTelemetry.update(telemetryM);
    }

    @Override
    public void start() {
        autoAlignSystem = new AutoAlignSystem(allianceColor); // INITIALIZE AUTO ALIGN SYSTEM
        autoAlignSystem.PedroSetUp(follower);
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
        panelsTelemetry.update(telemetryM); // update all telemetry

        // Clear cached data from control hubs for fresh readings
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        if (!automatedDrive) {
            //Make the last parameter false for field-centric, true for robot centric
            //This is the normal version to use in the TeleOp
            follower.setTeleOpDrive(
                    applyPolynomialToDriveInputs(gamepad1.left_stick_y),
                    applyPolynomialToDriveInputs(gamepad1.left_stick_x),
                    applyPolynomialToDriveInputs(gamepad1.right_stick_x),
                    robotCentric // robot centric/field centric
            );
        }

        // Update pedro pose if april tag detected via limelight
        Pose camPose = getRobotPoseFromCamera();
        if (camPose != null) follower.setPose(camPose);

        // IMU Reset
        if (gamepad1.dpadUpWasPressed()) {
            follower.setPose(follower.getPose().setHeading(90)); // Facing Up
        }
        if (gamepad1.dpadDownWasPressed()) {
            follower.setPose(allianceColor == RED ? Constants.RED_SHOOTING_FRONT : Constants.BLUE_SHOOTING_FRONT); // Reset to front launching pose
        }

        //Automated Path Following
        if (gamepad1.aWasPressed()) {
            // Find which launch zone is closer
            Pose backLaunchZone = allianceColor == RED ?
                    Constants.RED_SHOOTING_BACK : Constants.BLUE_SHOOTING_BACK;
            Pose frontLaunchZone = allianceColor == RED ?
                    Constants.RED_SHOOTING_FRONT : Constants.BLUE_SHOOTING_FRONT;
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
            // round to nearest integer
            if (heading - Math.floor(heading) < 0.5) {
                heading = Math.floor(heading);
            } else {
                heading = Math.ceil(heading);
            }
            heading = heading * 90; // go back to before
            heading = heading % 360; // Apply modulo operator to convert 360 to 0.
            // Follow path to the parking zone
            Pose parkingPose = allianceColor == RED ?
                    Constants.RED_PARKING : Constants.BLUE_PARKING;
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
        if (gamepad1.bWasPressed() || !follower.isBusy()) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // Auto Alignment using AutoAlignSystem
        long nowNs = System.nanoTime();
        dt = (nowNs - lastTimeNs) / 1e9;
        lastTimeNs = nowNs;
        // Manual-stick override: if driver is giving significant rotation input, disable auto-align
        if (gamepad1.x && Math.abs(gamepad1.right_stick_x) <= 0.12) {
            automatedDrive = true; // turning using pedro pathing.
            // Use auto align system with calculated dt
            autoAlignSystem.turnAutoAlign(dt);
        } else {
            // Not auto-aiming, ensure normal teleop drive continues
            // No additional action needed as setTeleOpDrive handles this
            automatedDrive = false;
            autoAlignSystem.resetPIDController();
        }

        // Launcher and Intake
        if (gamepad1.leftBumperWasPressed()) {
            // Run launcher
            if (!launcherOn) {
                double velocity = IsBackLaunchZoneCloser() ?
                        Constants.TARGET_VELOCITY_BACK_LAUNCH_ZONE : Constants.TARGET_VELOCITY_FRONT_LAUNCH_ZONE;
                double min_velocity = velocity - VELOCITY_TOLERANCE;
                // Set launcher velocity based on launch zone
                launcher.setVelocity(min_velocity);
                launcher.setVelocity(velocity);
                launcherOn = true;
            } else {
                launcher.setVelocity(STOP_VELOCITY);
                launcherOn = false;
            }
        }
        // ALL BALL LAUNCHING LOGIC - WITH VELOCITY CHECK
        if (gamepad1.right_bumper && launchingState==LaunchingState.IDLE) {

            launchingState = LaunchingState.LAUNCHER_SPINNING_UP;
        }

        if (launchingState == LaunchingState.LAUNCHER_SPINNING_UP) {
            // Wait for launcher to reach minimum velocity before starting transfer and intake
            double velocity = IsBackLaunchZoneCloser() ?
                    Constants.TARGET_VELOCITY_BACK_LAUNCH_ZONE : Constants.TARGET_VELOCITY_FRONT_LAUNCH_ZONE;
            double min_velocity = velocity - VELOCITY_TOLERANCE;

            if (launcher.getVelocity() >= min_velocity) {
                transfer.setPower(TRANSFER_UP_POSITION);
                intake.setPower(1);
                launchingState = LaunchingState.FIRING;
                transfer_timer.resetTimer();
            }
        }

        if (launchingState == LaunchingState.FIRING) {
            // Wait for RAPID_FIRE_TIME then reset everything
            if (transfer_timer.getElapsedTimeSeconds() * 1000 >= Constants.RAPID_FIRE_TIME) {
                transfer.setPower(Constants.TRANSFER_DOWN_POSITION);
                intake.setPower(0);
                launchingState = LaunchingState.IDLE;
            }
        }
        // INCREASE/DECREASE LAUNCHER VELOCITY
        if (gamepad1.rightStickButtonWasPressed()) {
            launcher.setVelocity(launcher.getVelocity()+Constants.INCREMENT_CHANGE_IN_VELOCITY);
        } else if (gamepad1.leftStickButtonWasPressed()) {
            launcher.setVelocity(launcher.getVelocity()-Constants.INCREMENT_CHANGE_IN_VELOCITY);
        }
        // INTAKE CONTROL - Only manual control when not firing
        if (launchingState == LaunchingState.IDLE) {
            intake.setPower(Math.max(Math.min(
                    gamepad1.right_trigger - gamepad1.left_trigger * 1.1, 1
            ), -1)); //Counteract imperfect intake power
        }
        panelsTelemetry.addData("Angle (in degrees)", limelight.tx); // LLScore is negative/positive
        panelsTelemetry.addData("Launcher Velocity", launcher.getVelocity());
        panelsTelemetry.addData("Launching State", launchingState);
        if (launchingState != LaunchingState.IDLE) panelsTelemetry.addData("Launch Timer (ms)", transfer_timer.getElapsedTimeSeconds()*1000);
    }
    private Pose getRobotPoseFromCamera() {
        // Thanks to team 20367 RMS Overdrive for this code snippet
        try {
            LLResult result = limelight.result;
            // Get robot pose from Limelight (in FTC coordinates)
            Pose3D botpose = result.getBotpose();

            // Check if pose data is valid
            if (botpose == null) {
                return null;
            }

            // Check if we have valid AprilTag detections
            if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
                return null;
            }

            // Parse Pose3D string representation to extract coordinates
            String poseString = botpose.toString();
            double x = 0.0, y = 0.0, heading = 0.0;

            try {
                // Extract x, y, and yaw (heading) from the pose string
                String[] parts = poseString.replaceAll("[{}]", "").split(",");
                for (String part : parts) {
                    String[] keyValue = part.split("=");
                    if (keyValue.length == 2) {
                        String key = keyValue[0].trim();
                        double value = Double.parseDouble(keyValue[1].trim());

                        switch (key) {
                            case "x":
                                x = value;
                                break;
                            case "y":
                                y = value;
                                break;
                            case "yaw":
                                heading = value;
                                break;
                        }
                    }
                }
            } catch (Exception parseException) {
                telemetry.addData("ERROR", "Failed to parse Pose3D: " + parseException.getMessage());
                return null;
            }

            // Validate the coordinates
            if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading) ||
                    Double.isInfinite(x) || Double.isInfinite(y) || Double.isInfinite(heading)) {
                return null;
            }

            // Check for reasonable field bounds
            if (Math.abs(x) > 200 || Math.abs(y) > 200) {
                return null;
            }

            // Create Pose in FTC coordinate system
            Pose ftcPose = new Pose(
                    x,      // X position in inches
                    y,      // Y position in inches
                    heading, // Heading in radians
                    FTCCoordinates.INSTANCE
            );

            // Convert from FTC coordinates to PedroPathing coordinates
            Pose pedroPose = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

            // Apply fusion with current PedroPathing pose for smoother corrections
            if (follower == null) {
                return pedroPose;
            }

            Pose currentPose = follower.getPose();

            // Distance check to prevent large jumps
            double distanceFromCurrent = Math.sqrt(
                    Math.pow(pedroPose.getX() - currentPose.getX(), 2) +
                            Math.pow(pedroPose.getY() - currentPose.getY(), 2)
            );

            // If the vision correction is too far from current pose, reject it
            if (distanceFromCurrent > 50.0) { // 50 inch threshold
                return null;
            }

            // Use weighted average for smoother corrections (70% current, 30% vision)
            double fusionWeight = 0.3; // 30% vision, 70% current

            double fusedX = currentPose.getX() * (1 - fusionWeight) + pedroPose.getX() * fusionWeight;
            double fusedY = currentPose.getY() * (1 - fusionWeight) + pedroPose.getY() * fusionWeight;
            double fusedHeading = currentPose.getHeading() * (1 - fusionWeight) + pedroPose.getHeading() * fusionWeight;

            return new Pose(fusedX, fusedY, fusedHeading);

        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to convert Limelight pose: " + e.getMessage());
            return null;
        }
    }
    public boolean IsBackLaunchZoneCloser() {
        Pose backLaunchZone = allianceColor == RED ?
                Constants.RED_SHOOTING_BACK : Constants.BLUE_SHOOTING_BACK;
        Pose frontLaunchZone = allianceColor == RED ?
                Constants.RED_SHOOTING_FRONT : Constants.BLUE_SHOOTING_FRONT;
        return getDistanceFromTwoPosesPP(follower.getPose(), backLaunchZone) <
                getDistanceFromTwoPosesPP(follower.getPose(), frontLaunchZone);
    }

    public double getDistanceFromTwoPosesPP(@NonNull Pose pose1, @NonNull Pose pose2) {return Math.hypot(pose2.getX() - pose1.getX(), pose2.getY() - pose1.getY());}
}