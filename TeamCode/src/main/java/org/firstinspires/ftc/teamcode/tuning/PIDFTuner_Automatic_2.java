package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@SuppressWarnings("SpellCheckingInspection")
@TeleOp(name = "Flywheel Automatic PIDF Tuner", group = "Tests")
public class PIDFTuner_Automatic_2 extends LinearOpMode {

    DcMotorEx flywheelL;
    DcMotorEx flywheelR;

    /* ======================== */
    /* ===== START PIDF ====== */
    /* ======================== */

    double kP_L = 0.002,  kI_L = 0.0, kD_L = 0.0001, kF_L = 0.05;
    double kP_R = 0.002,  kI_R = 0.0, kD_R = 0.0001, kF_R = 0.05;

    static final double[] TARGET_VELOCITIES = {800, 1000, 1200, 1400};

    static final int SAMPLE_TIME_MS = 1000;
    static final double VELOCITY_TOLERANCE = 25;

    @Override
    public void runOpMode() {

        flywheelL = hardwareMap.get(DcMotorEx.class, "OutakeMotorLeft");
        flywheelR = hardwareMap.get(DcMotorEx.class, "OutakeMotorRight");

        flywheelR.setDirection(DcMotorSimple.Direction.REVERSE);


        setup(flywheelL);
        setup(flywheelR);

        telemetry.addLine("Independent Dual Flywheel PIDF Tuner Ready");
        telemetry.update();
        waitForStart();

        tuneF(flywheelL, true);
        tuneF(flywheelR, false);

        tuneP();
        tuneD();
        tuneI();   // optional

        applyPIDF();

        telemetry.addLine("FINAL PIDF VALUES");
        telemetry.addData("L P I D F", "%.5f %.5f %.5f %.5f", kP_L, kI_L, kD_L, kF_L);
        telemetry.addData("R P I D F", "%.5f %.5f %.5f %.5f", kP_R, kI_R, kD_R, kF_R);
        telemetry.update();

        sleep(7000);
    }

    /* ======================== */
    /* ===== SETUP ===========
    /* ======================== */
    void setup(DcMotorEx m) {
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    void applyPIDF() {
        flywheelL.setVelocityPIDFCoefficients(kP_L, kI_L, kD_L, kF_L);
        flywheelR.setVelocityPIDFCoefficients(kP_R, kI_R, kD_R, kF_R);
    }

    /* ======================== */
    /* ===== F TUNING ======== */
    /* ======================== */
    void tuneF(DcMotorEx motor, boolean left) {
        telemetry.addLine(left ? "Tuning kF LEFT" : "Tuning kF RIGHT");
        telemetry.update();

        for (int i = 0; i < 8 && opModeIsActive(); i++) {

            double errorSum = 0;

            for (double target : TARGET_VELOCITIES) {
                applyPIDF();
                motor.setVelocity(target);
                sleep(SAMPLE_TIME_MS);
                errorSum += target - motor.getVelocity();
            }

            double avgError = errorSum / TARGET_VELOCITIES.length;

            if (Math.abs(avgError) < VELOCITY_TOLERANCE) break;

            if (left)
                kF_L += avgError > 0 ? 0.0015 : -0.0008;
            else
                kF_R += avgError > 0 ? 0.0015 : -0.0008;

            telemetry.addData("Avg Error", avgError);
            telemetry.update();
        }
    }

    /* ======================== */
    /* ===== P TUNING ======== */
    /* ======================== */
    void tuneP() {
        telemetry.addLine("Tuning kP (independent)");
        telemetry.update();

        for (int i = 0; i < 10 && opModeIsActive(); i++) {

            double oscL = measureOsc(flywheelL);
            double oscR = measureOsc(flywheelR);

            if (oscL < 80) kP_L += 0.00025;
            if (oscR < 80) kP_R += 0.00025;

            if (oscL > 120) kP_L *= 0.75;
            if (oscR > 120) kP_R *= 0.75;

            applyPIDF();

            telemetry.addData("kP_L", kP_L);
            telemetry.addData("kP_R", kP_R);
            telemetry.update();
        }
    }

    /* ======================== */
    /* ===== D TUNING ======== */
    /* ======================== */
    void tuneD() {
        telemetry.addLine("Tuning kD (independent)");
        telemetry.update();

        for (int i = 0; i < 8 && opModeIsActive(); i++) {

            double oscL = measureOsc(flywheelL);
            double oscR = measureOsc(flywheelR);

            if (oscL > 40) kD_L += 0.00005;
            if (oscR > 40) kD_R += 0.00005;

            applyPIDF();

            telemetry.addData("kD_L", kD_L);
            telemetry.addData("kD_R", kD_R);
            telemetry.update();
        }
    }

    /* ======================== */
    /* ===== I (OPTIONAL) ==== */
    /* ======================== */
    void tuneI() {
        double Lerror = 0, Rerror = 0;

        for (double t : TARGET_VELOCITIES) {
            applyPIDF();
            flywheelL.setVelocity(t);
            flywheelR.setVelocity(t);
            sleep(SAMPLE_TIME_MS);

            Lerror += Math.abs(t - flywheelL.getVelocity());
            Rerror += Math.abs(t - flywheelR.getVelocity());
        }

        if (Lerror > 140) {
            kI_L = 0.00002;
        }
        if (Rerror > 140) {
            kI_R = 0.00002;
        }
    }

    /* ======================== */
    /* ===== OSCILLATION ===== */
    /* ======================== */
    double measureOsc(DcMotorEx motor) {
        double min = Double.MAX_VALUE;
        double max = 0;

        for (int i = 0; i < 15; i++) {
            double v = motor.getVelocity();
            min = Math.min(min, v);
            max = Math.max(max, v);
            sleep(35);
        }
        return max - min;
    }
}