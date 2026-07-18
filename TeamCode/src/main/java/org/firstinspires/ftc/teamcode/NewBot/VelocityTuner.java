package org.firstinspires.ftc.teamcode.NewBot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode.Systems.RGBIndicator;

import java.util.List;

@TeleOp
public class VelocityTuner extends LinearOpMode {
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime launcherTimer = new ElapsedTime();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        LimelightSystem limelight = new LimelightSystem(hardwareMap);
        RGBIndicator rgbIndicator = new RGBIndicator(hardwareMap);
        rgbIndicator.setColor(RGBIndicator.Color.VIOLET);
        Constants.AllianceColor allianceColor = Constants.AllianceColor.RED;

        while (opModeInInit()) {
            telemetry.addLine("Press X to switch alliance");
            telemetry.addData("Alliance selected", allianceColor);
            rgbIndicator.setColor(allianceColor == Constants.AllianceColor.RED ? RGBIndicator.Color.RED : RGBIndicator.Color.BLUE);
            if (gamepad1.xWasReleased()) {
                allianceColor = allianceColor.switchColors();
            }
            telemetry.update();
        }

        Follower follower = Constants.createPedroFollower(hardwareMap);
        follower.setStartingPose(new Pose());

        DcMotorEx OuttakeMotor = hardwareMap.get(DcMotorEx.class, "OuttakeMotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor transferMotor = hardwareMap.dcMotor.get("transferMotor");

        OuttakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        OuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        OuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        OuttakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.LaunchPIDF);

        AutoAlignSystem autoAlignSystem = new AutoAlignSystem(allianceColor);
        autoAlignSystem.LimelightSetUp(limelight);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        limelight.start();
        telemetry.addData("Status", "Running");
        telemetry.update();

        boolean shouldUseLimelightAutoAlign = true;
        boolean shouldAutoAlign = false;
        boolean shouldKeepLauncherActive = false;
        int launcherStage = 0;
        double setTargetVelocity = 1000;
        double distance = 0;

        long lastTime = System.nanoTime();

        while (opModeIsActive()) {
            limelight.LLUpdate();
            rgbIndicator.updateUsingLL(limelight);

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            if (gamepad1.xWasPressed()) {
                allianceColor = allianceColor.switchColors();
                autoAlignSystem = new AutoAlignSystem(allianceColor);
                autoAlignSystem.LimelightSetUp(limelight);
            }
            if (gamepad1.dpadUpWasPressed()) {
                setTargetVelocity += 20;
            }
            if (gamepad1.dpadDownWasPressed()) {
                setTargetVelocity -= 20;
            }

            LLResultTypes.FiducialResult fiducialResult = limelight.getResultForTag(allianceColor.getTagID());
            if (fiducialResult != null) {
                distance = limelight.calculateDistance(fiducialResult);
                if (distance == 0) {
                    distance = limelight.dist;
                }
            }

            double y = -gamepad1.left_stick_y * 1.01;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x * 1.01;

            y *= Math.abs(y);
            x *= Math.abs(x);
            rx *= Math.abs(rx * 0.65);

            long currentTime = System.nanoTime();
            double deltaTime = (currentTime - lastTime) / 1e9;
            lastTime = currentTime;

            if (gamepad1.back || gamepad2.back) {
                shouldAutoAlign = true;
            }
            if (gamepad1.backWasReleased() || gamepad2.backWasReleased()) {
                shouldAutoAlign = false;
            }
            if (shouldAutoAlign) {
                rx = 0;
                if (!(rgbIndicator.getPWM() == RGBIndicator.GREEN_PWM) && shouldUseLimelightAutoAlign) {
                    rx = autoAlignSystem.getTurningPowerLimelight(deltaTime);
                } else {
                    shouldAutoAlign = false;
                }
            }

            follower.setTeleOpDrive(y, -x, -rx, false);
            follower.update();

            if (gamepad1.y || gamepad2.y) {
                switch (launcherStage) {
                    case 0:
                        shouldAutoAlign = true;
                        OuttakeMotor.setVelocity(setTargetVelocity);
                        launcherStage = 1;
                        launcherTimer.reset();
                        break;
                    case 1:
                        OuttakeMotor.setVelocity(setTargetVelocity);
                        boolean speedOk = OuttakeMotor.getVelocity() >= setTargetVelocity - Constants.VELOCITY_TOLERANCE
                                && (OuttakeMotor.getVelocity() <= setTargetVelocity + Constants.VELOCITY_TOLERANCE);
                        boolean autoAlignOk = rgbIndicator.getPWM() == RGBIndicator.GREEN_PWM;
                        boolean autoAlignSemiOk = limelight.getLLScore() < 4 && launcherTimer.seconds() > 3;
                        if ((speedOk && (autoAlignOk || autoAlignSemiOk)) || gamepad1.left_bumper) {
                            launcherStage = 2;
                            launcherTimer.reset();
                        }
                        break;
                    case 2:
                        intakeMotor.setPower(1);
                        transferMotor.setPower(Constants.TRANSFER_UP_POSITION);
                        break;
                    default:
                        launcherStage = 0;
                        launcherTimer.reset();
                        break;
                }
            } else if (gamepad1.a || gamepad2.a) {
                switch (launcherStage) {
                    case 0:
                        OuttakeMotor.setVelocity(setTargetVelocity);
                        launcherStage = 1;
                        launcherTimer.reset();
                        break;
                    case 1:
                        OuttakeMotor.setVelocity(setTargetVelocity);
                        boolean speedOk = OuttakeMotor.getVelocity() >= setTargetVelocity - Constants.VELOCITY_TOLERANCE
                                && (OuttakeMotor.getVelocity() <= setTargetVelocity + Constants.VELOCITY_TOLERANCE);
                        if (speedOk || gamepad1.left_bumper) {
                            launcherStage = 2;
                            launcherTimer.reset();
                        }
                        break;
                    case 2:
                        intakeMotor.setPower(1);
                        transferMotor.setPower(Constants.TRANSFER_UP_POSITION);
                        break;
                    default:
                        launcherStage = 0;
                        launcherTimer.reset();
                        break;
                }
            }

            if (((gamepad1.yWasReleased() || gamepad2.yWasReleased() || gamepad1.aWasReleased() || gamepad2.aWasReleased()) && launcherStage == 2)
                    || gamepad1.b || gamepad2.b) {
                if (!shouldKeepLauncherActive) {
                    OuttakeMotor.setVelocity(Constants.STOP_VELOCITY);
                }
                launcherStage = 0;
                transferMotor.setPower(Constants.TRANSFER_DOWN_POSITION);
                intakeMotor.setPower(0);
                shouldAutoAlign = false;
            }

            if (launcherStage != 2) {
                double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
                intakePower += gamepad2.right_trigger - gamepad2.left_trigger;
                intakePower = Math.max(-1, Math.min(1, intakePower * 1.2));
                intakeMotor.setPower(intakePower);
                if (Math.abs(intakePower) >= 0.3) {
                    transferMotor.setPower(-0.8 * Math.abs(intakePower));
                } else {
                    transferMotor.setPower(0);
                }
            }

            if (shouldKeepLauncherActive) {
                OuttakeMotor.setVelocity(setTargetVelocity);
            }

            if (gamepad1.dpadRightWasPressed()) {
                shouldKeepLauncherActive = !shouldKeepLauncherActive;
            }

            telemetry.addData("Alliance Color", allianceColor);
            telemetry.addData("LL Can see goal", autoAlignSystem.LLCanSeeGoal());
            telemetry.addData("RGB Indicator Color", rgbIndicator.getPWM() == RGBIndicator.GREEN_PWM ? "GREEN" : rgbIndicator.getPWM() == RGBIndicator.ORANGE_PWM ? "ORANGE" : rgbIndicator.getPWM() == RGBIndicator.YELLOW_PWM ? "YELLOW" : rgbIndicator.getPWM() == RGBIndicator.BLACK_PWM ? "BLACK" : rgbIndicator.getPWM() == RGBIndicator.RED_PWM ? "RED" : ("UNKNOWN COLOR (PWM:" + rgbIndicator.getPWM() + ")"));
            telemetry.addData("Launcher Stage", launcherStage);
            telemetry.addData("Intake Motor power", intakeMotor.getPower());
            telemetry.addData("Transfer Motor power", transferMotor.getPower());
            telemetry.addData("Outake Motor Velocity", OuttakeMotor.getVelocity());
            telemetry.addData("Target Velocity", setTargetVelocity);
            telemetry.addData("Tag ID", limelight.tid);
            telemetry.addData("Is fiducial null", fiducialResult == null);
            telemetry.addData("Distance", distance);
            telemetry.addData("Turning Power (RX only, not applied)", rx);
            telemetry.addData("Driver Y", y);
            telemetry.addData("Driver X", x);
            telemetry.addData("Should Keep Launcher Active", shouldKeepLauncherActive);
            telemetry.update();
        }



    }


}
