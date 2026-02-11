package org.firstinspires.ftc.teamcode.NewBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@SuppressWarnings("SpellCheckingInspection")
@TeleOp(name = "Flywheel Automatic PIDF Tuner (1 flywheel)", group = "Tests")
public class PIDFTuner_Automatic extends LinearOpMode {

    DcMotorEx flywheel;
    String flywheel_name = "OuttakeMotor";

    // ===== STARTING PIDF =====
    double kP = 300;
    double kI = 0.0;        // start disabled
    double kD = 0.0001;
    double kF = 10;

    static final double VELOCITY_TOLERANCE = 35;
    static final int SAMPLE_TIME_MS = 2000;

    static final double[] TARGET_VELOCITIES = {
            600, 900, 1100, 1200
    };

    @Override
    public void runOpMode() {

        flywheel = hardwareMap.get(DcMotorEx.class, flywheel_name);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        tuneFMulti();
        tunePMulti();
        tuneDMulti();
        tuneIMulti();   // optional, safe

        applyPIDF(); // safe

        flywheel.setVelocity(0); // To tell the user that tuning is done

        telemetry.addLine("FINAL PIDF VALUES");
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.addData("kF", kF);
        telemetry.update();

        while (opModeIsActive()) {
            if (gamepad1.b) {
                break;
            }
        }
    }

    /* ======================== */
    /* ===== F TUNING ======== */
    /* ======================== */
    void tuneFMulti() {
        telemetry.addLine("Tuning F (multi-speed)");
        telemetry.update();

        for (int iter = 0; iter < 8 && opModeIsActive(); iter++) {

            double totalError = 0;

            for (double target : TARGET_VELOCITIES) {
                applyPIDF();
                flywheel.setVelocity(target);
                sleep(SAMPLE_TIME_MS);

                totalError += (target - flywheel.getVelocity());
            }

            double avgError = totalError / TARGET_VELOCITIES.length;

            telemetry.addData("kF", kF);
            telemetry.addData("Avg Error", avgError);
            telemetry.update();

            if (Math.abs(avgError) < VELOCITY_TOLERANCE) break;

            kF += avgError > 0 ? 0.0015 : -0.0008;
        }
    }

    /* ======================== */
    /* ===== P TUNING ======== */
    /* ======================== */
    void tunePMulti() {
        telemetry.addLine("Tuning P (multi-speed)");
        telemetry.update();

        for (int iter = 0; iter < 10 && opModeIsActive(); iter++) {

            double maxOscillation = 0;

            for (double target : TARGET_VELOCITIES) {
                applyPIDF();
                flywheel.setVelocity(target);
                sleep(SAMPLE_TIME_MS);
                maxOscillation = Math.max(maxOscillation, measureOscillation());
            }

            telemetry.addData("kP", kP);
            telemetry.addData("Oscillation", maxOscillation);
            telemetry.update();

            if (maxOscillation > 100) {
                kP *= 0.75;
                break;
            }

            kP += 0.00025;
        }
    }

    /* ======================== */
    /* ===== D TUNING ======== */
    /* ======================== */
    void tuneDMulti() {
        telemetry.addLine("Tuning D");
        telemetry.update();

        for (int iter = 0; iter < 8 && opModeIsActive(); iter++) {

            double maxOscillation = 0;

            for (double target : TARGET_VELOCITIES) {
                applyPIDF();
                flywheel.setVelocity(target);
                sleep(SAMPLE_TIME_MS);
                maxOscillation = Math.max(maxOscillation, measureOscillation());
            }

            telemetry.addData("kD", kD);
            telemetry.addData("Oscillation", maxOscillation);
            telemetry.update();

            if (maxOscillation < 40) break;

            kD += 0.00005;
        }
    }

    /* ======================== */
    /* ===== I TUNING (SAFE) == */
    /* ======================== */
    void tuneIMulti() {
        telemetry.addLine("Checking need for I");
        telemetry.update();

        double totalSteadyError = 0;

        for (double target : TARGET_VELOCITIES) {
            applyPIDF();
            flywheel.setVelocity(target);
            sleep(SAMPLE_TIME_MS);
            totalSteadyError += Math.abs(target - flywheel.getVelocity());
        }

        if (totalSteadyError < 40) return; // I not needed

        kI = 0.0000005 * totalSteadyError; // conservative
    }

    /* ======================== */
    /* ===== HELPERS ========= */
    /* ======================== */
    void applyPIDF() {
        flywheel.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    double measureOscillation() {
        double min = Double.MAX_VALUE;
        double max = 0;

        for (int i = 0; i < 15; i++) {
            double v = flywheel.getVelocity();
            min = Math.min(min, v);
            max = Math.max(max, v);
            sleep(35);
        }
        return max - min;
    }
}