package org.firstinspires.ftc.teamcode.NewBot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode.Systems.RGBIndicator;


@SuppressWarnings("SpellCheckingInspection")
@TeleOp(name = "NewBotTeleOp", group = "TeleOp")// Name and Group
public class NewBotTeleOp extends LinearOpMode {

    boolean Gamepad2Driving = false, launcherOn= false, testingStartPose = false;
    // declaring our PIDF tuning values
    double setTargetVelocity = 0;
    double setMinVelocity = 0;
    private PedroDrive follower;
    Constants.AllianceColor allianceColor;
    LimelightSystem limelight;
    AutoAlignSystem autoAlignSystem;
    RGBIndicator rgbIndicator;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = new LimelightSystem(hardwareMap);
        rgbIndicator = new RGBIndicator(hardwareMap);
        rgbIndicator.setColor(RGBIndicator.Color.VIOLET);
        allianceColor = Constants.AllianceColor.RED;
        while (opModeInInit())
        {
            telemetry.addLine("Press X to switch alliance, A to switch drivers, B to switch between testing start pose and normal start pose");
            telemetry.addLine("Press X for Red Alliance, Y for Blue Alliance");
            telemetry.addLine("Press A for Start A Driver, B for Start B Driver");
            telemetry.addLine((Gamepad2Driving ? "Gamepad B" : "Gamepad A" )+ " Driving");
            telemetry.addData("Alliance selected: ", allianceColor);
            // This method is called repeatedly during the init phase
            if (gamepad1.aWasPressed()) {
                Gamepad2Driving = !Gamepad2Driving;
            }
            if (gamepad1.bWasPressed()) {
                testingStartPose = !testingStartPose;
            }
            if (gamepad1.xWasReleased()) {
                allianceColor = allianceColor.switchColors();
            }


            telemetry.update();
        }

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get(Constants.pedroMecanumDriveConstants.leftFrontMotorName);
        DcMotor backLeftMotor = hardwareMap.dcMotor.get(Constants.pedroMecanumDriveConstants.leftRearMotorName);
        DcMotor frontRightMotor = hardwareMap.dcMotor.get(Constants.pedroMecanumDriveConstants.rightFrontMotorName);
        DcMotor backRightMotor = hardwareMap.dcMotor.get(Constants.pedroMecanumDriveConstants.rightRearMotorName);
        // Using DcMotorEx instead of DcMotor to use PID controller
        DcMotorEx OuttakeMotor = hardwareMap.get(DcMotorEx.class,"OuttakeMotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor transferMotor = hardwareMap.dcMotor.get("transferMotor");

        follower = new PedroDrive(hardwareMap,
                // calculate pose
                testingStartPose ? new Pose(120, 36, 180) : ( // Testing Pose (near our table)
                        // X: line between second to last and last field tile
                        // Y: Middle of second to last field tile
                        allianceColor == Constants.AllianceColor.RED ?
                                NewBotAutoRed.finalPose :
                                NewBotAutoBlue.finalPose
                        )
                );

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        OuttakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        OuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        OuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        OuttakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.LaunchPIDF);

        autoAlignSystem = new AutoAlignSystem(allianceColor);

        autoAlignSystem.LimelightSetUp(limelight);

        telemetry.addData("Status", "Initialized");telemetry.update();
        waitForStart();

        if (isStopRequested()) return;
        limelight.start(0);
        telemetry.addData("Status", "Running");

        // --- NEW VARIABLES FOR BUTTON TOGGLES ---
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;
        // ----------------------------------------

        // calculate dt for turning with limelight
        long lastTime = System.nanoTime();
        long currentTime;
        double deltaTime;

        while (opModeIsActive()) {
            follower.update();
            limelight.LLUpdate();
            rgbIndicator.updateUsingLL(limelight);


            double y, x, rx;
            if (Gamepad2Driving) {
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
            rx *= Math.abs(rx * 0.75);

            // Calculate dt
            currentTime = System.nanoTime();
            deltaTime = (currentTime - lastTime) / 1e9; // convert to seconds
            lastTime = currentTime; // Update lastTime each loop so dt is meaningful


            if (gamepad1.back || gamepad2.back) {
                if (autoAlignSystem.LLCanSeeGoal()) {
                    follower.correctPose(PedroDrive.getPedroPoseFromLL(limelight, allianceColor));
                }
                rx = autoAlignSystem.getTurningPowerPedro(follower.getPose(), deltaTime);
            }
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = 0.9 *(y + x + rx) / denominator;
            double backLeftPower = 0.9 *(y - x + rx) / denominator;
            double frontRightPower = 0.9 *(y - x - rx) / denominator;
            double backRightPower = 0.9 * (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            LLResultTypes.FiducialResult fiducialResult = limelight.getResultForTag(allianceColor == Constants.AllianceColor.RED? 24 : 20);
            double distance = LimelightSystem.calculateDistanceFromRelativePose(fiducialResult);
            if (gamepad1.y || gamepad2.y) {
                launcherOn = true;
                setTargetVelocity = VelocityCalculator.NEWBOT.calculateVelocity(distance); // use new velocity calculator
                setMinVelocity = setTargetVelocity - Constants.VELOCITY_TOLERANCE;
                OuttakeMotor.setVelocity(setMinVelocity);
                OuttakeMotor.setVelocity(setTargetVelocity);

            }


            if ((gamepad1.x || gamepad2.x) && launcherOn) {
                intakeMotor.setPower(1);
                transferMotor.setPower(Constants.TRANSFER_UP_POSITION);
            } else {

                // In-take
                double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
                intakePower += gamepad2.right_trigger - gamepad2.left_trigger;
                intakePower = intakePower * 1.5;
                intakePower = Math.max(-1, Math.min(1, intakePower)); // Clip to [-1, 1]
                intakeMotor.setPower(intakePower);
                if (Math.abs(intakePower) >= 0.3) {
                    transferMotor.setPower(-Constants.TRANSFER_UP_POSITION*Math.abs(intakePower));
                } else {
                    transferMotor.setPower(0);
                }
            }
            if (gamepad1.b || gamepad2.b) {
                // new- safe braking (like Car braking?)
                if (launcherOn) {OuttakeMotor.setVelocity(Constants.STOP_VELOCITY);setTargetVelocity=0;setMinVelocity=0;}
                else if (setTargetVelocity == 0) {OuttakeMotor.setVelocity(Constants.BREAK_STOP_VEL);setTargetVelocity=Constants.BREAK_STOP_VEL;setMinVelocity=0;}
                else if (setTargetVelocity == Constants.BREAK_STOP_VEL) {OuttakeMotor.setVelocity(Constants.STOP_VELOCITY);setTargetVelocity=0;setMinVelocity=0;}
                launcherOn = false;
                transferMotor.setPower(Constants.TRANSFER_DOWN_POSITION);
            }
            // check for B button release to stop the launcher and reset velocity variables (incase stuck at break-stop-vel)
            if (gamepad1.bWasReleased() || gamepad2.bWasReleased()) {OuttakeMotor.setVelocity(Constants.STOP_VELOCITY);setTargetVelocity=0;setMinVelocity=0;}

            // Check for Dpad Up (Increase Velocity)
            boolean currentDpadUp = gamepad1.dpad_up || gamepad2.dpad_up;
            if (currentDpadUp && !lastDpadUp) {
                setTargetVelocity += 25;
                setMinVelocity = setTargetVelocity - Constants.VELOCITY_TOLERANCE;
                OuttakeMotor.setVelocity(setTargetVelocity);
            }
            lastDpadUp = currentDpadUp;

            // Check for Dpad Down (Decrease Velocity)
            boolean currentDpadDown = gamepad1.dpad_down || gamepad2.dpad_down;
            if (currentDpadDown && !lastDpadDown) {
                setTargetVelocity -= 25;
                setMinVelocity = setTargetVelocity - Constants.VELOCITY_TOLERANCE;
                OuttakeMotor.setVelocity(setTargetVelocity);
            }
            lastDpadDown = currentDpadDown;

            // ------------------------------------
            telemetry.addData("POSE", follower.getPose());
            telemetry.addData("Alliance Color", allianceColor);
            telemetry.addData("Intake Motor power", intakeMotor.getPower());
            telemetry.addData("Transfer Motor power", transferMotor.getPower());
            telemetry.addData("Outake Motor Velocity:", OuttakeMotor.getVelocity());
            telemetry.addData("Target Velocity", setTargetVelocity);
            telemetry.addData("Min Velocity", setMinVelocity);
            telemetry.addData("Tag ID", limelight.tid);
            telemetry.addData("Distance", limelight.dist);
            telemetry.addData("Distance to goal", distance);
            telemetry.addData("Turning Power (RX)", rx);
            telemetry.update();
        }
    }
}