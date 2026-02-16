package org.firstinspires.ftc.teamcode.NewBot;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * PIDF controller with anti-windup, output clamping and optional feedforward based on setpoint velocity.
 */
@SuppressWarnings("SpellCheckingInspection")
public class PIDFController {
    private final double kP, kI, kD, kF;
    private final double minOutput, maxOutput;
    private final double integralLimit;

    private double integral = 0.0;
    private double lastError = 0.0;
    private boolean firstUpdate = true;

    private double tolerance = 0.5;
    private double lastOutput = 0.0;

    /**
     * Construct a PIDF controller.
     *
     * @param coefficients PIDF coefficients
     * @param minOutput minimum output after clamping
     * @param maxOutput maximum output after clamping
     * @param integralLimit absolute limit for the integral term
     */
    public PIDFController(PIDFCoefficients coefficients, double minOutput, double maxOutput, double integralLimit) {
        this.kP = coefficients.p;
        this.kI = coefficients.i;
        this.kD = coefficients.d;
        this.kF = coefficients.f;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        this.integralLimit = integralLimit;
    }

    /**
     * Backwards-compatible constructor (no feedforward).
     */
    public PIDFController(PIDCoefficients coefficients, double minOutput, double maxOutput, double integralLimit) {
        this(new PIDFCoefficients(coefficients.p,coefficients.i,coefficients.d,0.0), minOutput, maxOutput, integralLimit);
    }

    /**
     * Update controller with error and dt. No feedforward (kF * setpointVelocity = 0).
     *
     * @param error     current error (setpoint - measurement)
     * @param dtSeconds time step in seconds
     * @return controller output (clamped)
     */
    public double updatePIDF(double error, double dtSeconds) {
        return updatePIDF(error, dtSeconds, 0.0);
    }

    /**
     * Update controller with error, dt and optional setpoint velocity used for feedforward.
     *
     * @param error            current error (setpoint - measurement)
     * @param dtSeconds        time step in seconds
     * @param setpointVelocity velocity (units/sec) used for feedforward term
     * @return controller output (clamped)
     */
    public double updatePIDF(double error, double dtSeconds, double setpointVelocity) {
        //if (dtSeconds <= 0) return 0.0;
        if (firstUpdate) {
            lastError = error;
            firstUpdate = false;
        }

        // Proportional
        double p = kP * error;

        // Integral with anti-windup
        integral += error * dtSeconds;
        if (Math.abs(integral) > integralLimit) {
            integral = Math.signum(integral) * integralLimit;
        }
        double i = kI * integral;

        // Derivative
        double derivative = (error - lastError) / dtSeconds;
        double d = kD * derivative;

        lastError = error;

        // Feedforward (based on setpoint velocity)
        double f = kF * setpointVelocity;

        double output = p + i + d + f;

        // Clamp
        if (output > maxOutput) output = maxOutput;
        if (output < minOutput) output = minOutput;

        lastOutput = output;
        return output;
    }

    /**
     * Reset internal state (integral, derivative history).
     */
    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        firstUpdate = true;
        lastOutput = 0.0;
    }

    /**
     * Set tolerance for at-setpoint checks.
     */
    public void setTolerance(double tolerance) {
        this.tolerance = Math.max(0.0, tolerance);
    }

    /**
     * Return true if error is within configured tolerance.
     */
    public boolean atSetpoint(double error) {
        return Math.abs(error) <= tolerance;
    }

    /**
     * Return last computed output (useful for diagnostics).
     */
    public double getLastOutput() {
        return lastOutput;
    }
}
