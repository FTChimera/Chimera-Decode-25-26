package org.firstinspires.ftc.teamcode.NewBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;

@TeleOp(name = "Auto Align PD Tuner", group = "Tuning")
public class AutomaticHeadingAutoAlignPDTuner extends LinearOpMode {

    // ===== HARDWARE =====
    DcMotor frontLeft, frontRight, backLeft, backRight;

    // Limelight (replace with your actual Limelight class)
    LimelightSystem limelight;

    // ===== PD GAINS =====
    double kP = 0.1;
    double kD = 0.001;

    static final double MIN_OUTPUT = -0.6;
    static final double MAX_OUTPUT = 0.6;

    static final double TARGET_TOLERANCE = 0.5;  // degrees
    static final double MAX_OSCILLATION = 2.5;   // degrees swing

    PIDFController controller;

    @Override
    public void runOpMode() {

        initDrive();
        limelight = new LimelightSystem(hardwareMap);

        telemetry.addLine("PD Auto Align Tuner Ready");
        telemetry.update();
        waitForStart();
        limelight.start(0);
        tuneP();
        tuneD();

        telemetry.addLine("FINAL PD VALUES");
        telemetry.addData("kP", kP);
        telemetry.addData("kD", kD);
        telemetry.update();

        while (opModeIsActive()) {
            if (gamepad1.b || gamepad2.b) {
                break;
            }
        }
        requestOpModeStop();
        terminateOpModeNow();
    }

    /* ======================== */
    /* ===== TUNE P FIRST ==== */
    /* ======================== */
    void tuneP() {

        telemetry.addLine("Tuning kP...");
        telemetry.update();

        for (int i = 0; i < 10 && opModeIsActive(); i++) {

            controller = new PIDFController(
                    new PIDFCoefficients(kP, 0, 0, 0),
                    MIN_OUTPUT, MAX_OUTPUT, 0
            );

            double overshoot = runAlignmentTest();

            telemetry.addData("kP", kP);
            telemetry.addData("Overshoot", overshoot);
            telemetry.update();

            if (overshoot > MAX_OSCILLATION) {
                kP *= 0.7;
                break;
            }

            kP += 0.003;
        }
    }

    /* ======================== */
    /* ===== TUNE D NEXT ===== */
    /* ======================== */
    void tuneD() {

        telemetry.addLine("Tuning kD...");
        telemetry.update();

        for (int i = 0; i < 10 && opModeIsActive(); i++) {

            controller = new PIDFController(
                    new PIDFCoefficients(kP, 0, kD, 0),
                    MIN_OUTPUT, MAX_OUTPUT, 0
            );

            double oscillation = runAlignmentTest();

            telemetry.addData("kD", kD);
            telemetry.addData("Oscillation", oscillation);
            telemetry.update();

            if (oscillation < 0.8) break;

            kD += 0.001;
        }
    }

    /* ======================== */
    /* ===== RUN TEST ======== */
    /* ======================== */
    double runAlignmentTest() {

        controller.reset();
        controller.setTolerance(TARGET_TOLERANCE);

        double maxError = 0;

        long lastTime = System.nanoTime();

        while (opModeIsActive()) {
            limelight.LLUpdate();
            double tx = limelight.tx;  // horizontal offset
            double error = tx;

            long now = System.nanoTime();
            double dt = (now - lastTime) / 1e9;
            lastTime = now;

            double output = controller.updatePIDF(error, dt);

            setTurnPower(output);

            maxError = Math.max(maxError, Math.abs(error));

            if (controller.atSetpoint(error)) break;

            telemetry.addData("Error", error);
            telemetry.update();
        }

        setTurnPower(0);

        return maxError;
    }

    /* ======================== */
    /* ===== DRIVE CONTROL ==== */
    /* ======================== */
    void initDrive() {
        frontLeft  = hardwareMap.get(DcMotor.class, Constants.pedroMecanumDriveConstants.leftFrontMotorName);
        frontRight = hardwareMap.get(DcMotor.class, Constants.pedroMecanumDriveConstants.rightFrontMotorName);
        backLeft   = hardwareMap.get(DcMotor.class, Constants.pedroMecanumDriveConstants.leftRearMotorName);
        backRight  = hardwareMap.get(DcMotor.class, Constants.pedroMecanumDriveConstants.rightRearMotorName);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    void setTurnPower(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }
}