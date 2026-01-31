package org.firstinspires.ftc.teamcode2.TeleOp;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import static org.firstinspires.ftc.teamcode2.Systems.Consts.STOP_VELOCITY;
import static org.firstinspires.ftc.teamcode2.Systems.Consts.VELOCITY_TOLERANCE;
import static org.firstinspires.ftc.teamcode2.Systems.Consts.applyPolynomialToDriveInputs;

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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode2.Auto.Blue_Close;
import org.firstinspires.ftc.teamcode2.Auto.Blue_Far;
import org.firstinspires.ftc.teamcode2.Auto.Red_Close;
import org.firstinspires.ftc.teamcode2.Auto.Red_Far;
import org.firstinspires.ftc.teamcode2.Systems.AutoAlignSystem;
import org.firstinspires.ftc.teamcode2.Systems.Consts;
import org.firstinspires.ftc.teamcode2.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode2.Systems.RGBIndicator;
import org.firstinspires.ftc.teamcode2.pedroPathing.Constants;

import java.util.List;

@SuppressWarnings("SpellCheckingInspection")
@Configurable
@TeleOp(name = "Pedro_TeleOp", group = "TeleOp")
public class Pedro_TeleOp extends OpMode {
    /*
    * Note that all variables here are updated.
    * Make sure that every constant final variable is kept in Consts.java
    * FIGURE OUT HOW TO COMPLETE TELEOP
    */

    // set up bulk reading on sensors
    private List<LynxModule> allHubs;
    private LimelightSystem limelight; private AutoAlignSystem autoAlignSystem;
    private RGBIndicator rgbIndicator;
    private Follower follower;
    private Consts.AllianceColor allianceColor = Consts.AllianceColor.RED;public Consts.Auto auto = Consts.Auto.RED_CLOSE;
    // Removed redundant PID controller - now handled by AutoAlignSystem
    private boolean automatedDrive=false, launcherOn=false, robotCentric=true;
    private long lastTimeNs,nowNs;
    double dt;
    private enum LaunchingState {
        IDLE,
        GOING_UP,
        LAUNCHING,
        GOING_DOWN
    } private LaunchingState launchingState = LaunchingState.IDLE;
    protected MultipleTelemetry telemetryM;
    private TelemetryManager panelsTelemetry;
    public Pose startingPose;
    DcMotorEx launcher; DcMotor intake; CRServo pushServo;
    Timer Servo_timer;

    @Override
    public void init() {
        // set up bulk reading
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        limelight = new LimelightSystem(hardwareMap);
        rgbIndicator = new RGBIndicator(hardwareMap.get(Servo.class, "rgb"));
        pushServo = hardwareMap.get(CRServo.class, "push");
        Servo_timer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        telemetryM = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        ); // cannot combine panels telemetry with normal so need to send update from panels
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.dcMotor.get("intake");

        launcher.setZeroPowerBehavior(FLOAT);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Consts.LaunchPIDF);
        pushServo.setPower(Consts.SERVO_DOWN_POSITION);
    }
    @Override
    public void init_loop() {
        panelsTelemetry.addData("Auto (press A to switch)", auto);
        panelsTelemetry.addData("(press B to switch) Robot Centric is", robotCentric);
        if (gamepad1.aWasPressed()) {
            auto = auto.next();
            allianceColor = auto.getAllianceColor();
        }
        if (gamepad1.bWasPressed()) {
            robotCentric = !robotCentric;
        }
        // Set starting pose based on alliance color
        if (auto == Consts.Auto.RED_CLOSE) {
            startingPose = Red_Close.endPose; // Auto end pose is TeleOp start pose
        } else if (auto == Consts.Auto.BLUE_CLOSE) {
            startingPose = Blue_Close.endPose;
        } else if (auto == Consts.Auto.BLUE_FAR) {
            startingPose = Blue_Far.endPose;
        } else if (auto == Consts.Auto.RED_FAR) {
            startingPose = Red_Far.endPose;
        }

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
        // Auto Alignment using AutoAlignSystem
        nowNs = System.nanoTime();
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
        }

        // Launcher and Intake
        if (gamepad1.leftBumperWasPressed()) {
            // Run launcher
            if (!launcherOn) {
                double velocity = IsBackLaunchZoneCloser() ?
                        Consts.TARGET_VELOCITY_BACK_LAUNCH_ZONE : Consts.TARGET_VELOCITY_FRONT_LAUNCH_ZONE;
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
        // ALL SERVO LAUNCHING LOGIC
        if (gamepad1.right_bumper && launchingState==LaunchingState.IDLE) {
            // In IDLE, we don't do anything
            launchingState = LaunchingState.GOING_UP;
            Servo_timer.resetTimer();
        }

        if (launchingState==LaunchingState.GOING_UP) {
            // This is the Fire logic.
            double velocity = IsBackLaunchZoneCloser() ?
                    Consts.TARGET_VELOCITY_BACK_LAUNCH_ZONE : Consts.TARGET_VELOCITY_FRONT_LAUNCH_ZONE;
            double min_velocity = velocity - VELOCITY_TOLERANCE;
            // Wait until the launcher reaches past the velocity tolerance
            if (launcher.getVelocity() >= min_velocity) {
                pushServo.setPower(Consts.SERVO_UP_POSITION);
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
            pushServo.setPower(Consts.SERVO_DOWN_POSITION);
            launchingState = LaunchingState.IDLE;
            Servo_timer.resetTimer();
        }
        // INCREASE/DECREASE LAUNCHER VELOCITY
        if (gamepad1.rightStickButtonWasPressed()) {
            launcher.setVelocity(launcher.getVelocity()+25);
        } else if (gamepad1.leftStickButtonWasPressed()) {
            launcher.setVelocity(launcher.getVelocity()-25);
        }
        // INTAKE CONTROL
        intake.setPower(Math.max(Math.min(gamepad1.left_trigger - gamepad1.right_trigger *1.1,1),-1)); //Counteract imperfect intake power
        panelsTelemetry.addData("Angle (in degrees)", limelight.tx); // LLScore is negative/positive
        panelsTelemetry.addData("Launcher Velocity", launcher.getVelocity());
        panelsTelemetry.addData("Servo data", launchingState);
        if (launchingState != LaunchingState.IDLE) panelsTelemetry.addData("Servo Timer (ms)", Servo_timer.getElapsedTimeSeconds()/1000);
    }
    private Pose getRobotPoseFromCamera() {
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
        Pose backLaunchZone = allianceColor == Consts.AllianceColor.RED ?
                Consts.RED_SHOOTING_BACK : Consts.BLUE_SHOOTING_BACK;
        Pose frontLaunchZone = allianceColor == Consts.AllianceColor.RED ?
                Consts.RED_SHOOTING_FRONT : Consts.BLUE_SHOOTING_FRONT;
        return getDistanceFromTwoPosesPP(follower.getPose(), backLaunchZone) <
                getDistanceFromTwoPosesPP(follower.getPose(), frontLaunchZone);
    }

    public double getDistanceFromTwoPosesPP(Pose pose1, Pose pose2) {return Math.hypot(pose2.getX() - pose1.getX(), pose2.getY() - pose1.getY());}
}