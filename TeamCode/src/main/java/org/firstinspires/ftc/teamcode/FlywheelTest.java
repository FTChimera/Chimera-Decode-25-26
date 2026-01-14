package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


//notes:
/*
notes:
comments are made by me and are just reminders
this code is for the single flywheel design

 */
@TeleOp
public class FlywheelTest extends OpMode {
//defining all the motors
    public DcMotorEx outakeMotor;

    public double highVelocity = 1500;

    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double F = 0;

    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};//A bunch of numbers that make the tuning more and more presice

    int stepIndex = 1;// changes each of the stepSizes by an incremnt of one decimal place so that it gets more precise every time

    @Override
    public void init() {
        //init the motors
        outakeMotor = hardwareMap.get(DcMotorEx.class, "outakeMotor");
        outakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        outakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete");
    }

    @Override

    public void loop() {
        //get all game pad controls
        //set target volocity
        //telemetry updates

        if (gamepad1.yWasPressed()) {
            if(curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if  (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        outakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        outakeMotor.setVelocity(curTargetVelocity);

        double curVelocity = outakeMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);// what the target vel is
        telemetry.addData("Current Velocity", "%.2f", curVelocity);// what the acc vel is
        telemetry.addData("Error", "%.2f", error);// how much error there is
        telemetry.addLine("---------------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);//current vlaues for the p value
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);// current values for the f value
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);//current step sizes
    }
}
