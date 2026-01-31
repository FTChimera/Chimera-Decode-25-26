package org.firstinspires.ftc.teamcode2.Tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode2.Systems.Consts;

@SuppressWarnings("SpellCheckingInspection")
@TeleOp(name="Max Velocity Test", group="Tests")
public class MaxVelocityTest extends LinearOpMode {
    DcMotorEx motor;
    double currentVelocity;
    double maxVelocity = 0.0;


    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "launcher");
        waitForStart();
        while (opModeIsActive()) {
            currentVelocity = motor.getVelocity();
            if (gamepad1.a) {motor.setVelocity(Consts.TARGET_VELOCITY_BACK_LAUNCH_ZONE);}
            if (gamepad1.b) {motor.setVelocity(Consts.STOP_VELOCITY);}
            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            double Kf = maxVelocity==0? 0 : 32767 / maxVelocity;
            double Kp = Kf * 0.1;
            double Ki = Kp * 0.1;
            double Kd = 0;

            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.addLine("");
            telemetry.addLine("PIDF Coefficients:");
            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);
            telemetry.addData("Kf", Kf);
            telemetry.update();
        }
    }
}

