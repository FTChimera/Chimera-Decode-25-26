package org.firstinspires.ftc.teamcode.biobuzzTest;

import java.util.function.Function;
import java.util.function.Supplier;

public class PIDFController<T> {
    private PIDFCoefficients coefficients;
    private T target;
    Function<T, double[]> stateSupplier;
    Function<double[], T> inverseStateSupplier;
    private T output;
    private double[] lastError;
    private double[] integral;
    private boolean firstUpdate = false;
    public double dt;
    private double lastTime;
    public void setStateSupplier(Function<T,double[]> stateSupplier) {
        this.stateSupplier = stateSupplier;
    }
    public void setInverseStateSupplier(Function<double[],T> inverseStateSupplier) {
        this.inverseStateSupplier = inverseStateSupplier;
    }
    public PIDFController(PIDFCoefficients coefficients) {
        this.coefficients = coefficients;
    }

    public void setTarget(T target) {
        this.target = target;
    }

    public T getTarget() {
        return target;
    }

    public PIDFCoefficients getCoefficients() {
        return coefficients;
    }
    public void reset() {
        firstUpdate = false;
        integral = null;
        lastError = null;
        dt = 0;
        lastTime = 0;
    }

    public boolean atTarget() {
        if (!firstUpdate) return false;
        for (int i = 0; i < lastError.length; i++) {
            if (Math.abs(lastError[i]) > coefficients.tolerance[i]) return false;
        }
        return true;
    }


    /** Updates the PIDF Controller. Should be called periodically.
     * @param state the current reading from sensor or elsewhere
     * */
    public void update(T state) {
        try {
            if (!firstUpdate) {
                lastError = new double[stateSupplier.apply(state).length];
                integral = new double[stateSupplier.apply(state).length];
                lastTime = System.nanoTime();
                firstUpdate = true;
            }
            dt = (System.nanoTime() - lastTime) / 1e9;
            lastTime = System.nanoTime();
            double[] currentStateArray = stateSupplier.apply(state);
            double[] targetArray = stateSupplier.apply(target);

            double[] error = new double[currentStateArray.length];
            for (int i = 0; i < currentStateArray.length; i++) {
                error[i] = targetArray[i] - currentStateArray[i];
            }
            double[] outputArray = new double[error.length];
            for (int i = 0; i < error.length; i++) {
                integral[i] += error[i] * dt;
                // Anti-windup: clamp integral
                if (integral[i] > coefficients.integralLimit[i]) integral[i] = coefficients.integralLimit[i];
                if (integral[i] < -coefficients.integralLimit[i]) integral[i] = -coefficients.integralLimit[i];

                outputArray[i] = coefficients.p[i] * error[i];
                outputArray[i] += coefficients.d[i] * (error[i] - lastError[i]) / dt;
                outputArray[i] += coefficients.i[i] * integral[i];
                outputArray[i] += coefficients.f[i] * Math.signum(error[i]);
                if (atTarget()) outputArray[i] = 0;
            }
            this.output = inverseStateSupplier.apply(outputArray);
            lastError = error;
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public T getOutput() {
        return output;
    }

}
