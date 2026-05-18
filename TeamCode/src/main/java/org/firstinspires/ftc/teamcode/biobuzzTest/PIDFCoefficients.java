package org.firstinspires.ftc.teamcode.biobuzzTest;

public class PIDFCoefficients {
    public double[] p;
    public double[] i;
    public double[] d;
    public double[] f;
    public double[] tolerance;
    public double[] integralLimit;

    public PIDFCoefficients(double[] p, double[] i, double[] d, double[] f, double[] tolerance, double[] integralLimit) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.tolerance = tolerance;
        this.integralLimit = integralLimit;
    }

}
