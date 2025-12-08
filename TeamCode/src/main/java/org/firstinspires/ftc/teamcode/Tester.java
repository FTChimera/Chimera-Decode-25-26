package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Vision.aprilTagIndicatorTest;

@Configurable
@TeleOp(name="Tester Selectable")
public class Tester extends SelectableOpMode {
    public Tester() {
        super("Select an OpMode", s -> {
            s.folder("Launchers Test", l -> {
                l.add("Left PID Test", MaxVelocityPIDLeft::new);
                l.add("Right PID Test", MaxVelocityPIDRight::new);
            });
            s.folder("Limelight and RGB Indicator", l -> {
                l.add("April Tag Indicator Test", aprilTagIndicatorTest::new);
            });
        });
    }

    @TeleOp
    public static class MaxVelocityPIDLeft extends LinearOpMode {
        DcMotorEx motor;
        double currentVelocity, maxVelocity;
        @Override
        public void runOpMode() throws InterruptedException {
            motor = hardwareMap.get(DcMotorEx.class, "OutakeMotorLeft");
            boolean startPID = false;
            maxVelocity = 0;
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            while (opModeInInit()) {
                if (startPID) {
                    if (gamepad1.a) motor.setPower(1);
                    if (gamepad1.b) motor.setPower(0);
                    currentVelocity = motor.getVelocity();
                    if (maxVelocity < currentVelocity) maxVelocity=currentVelocity;
                    telemetry.addData("Max Velocity", maxVelocity);
                    telemetry.addData("Current Velocity", currentVelocity);
                } else {
                    telemetry.addData("BUTTON TO START PID", "Gamepad 1 A");
                    while (!gamepad1.a) idle();
                    startPID = true;
                }
                telemetry.update();
            }
            waitForStart();
            motor.setPower(0);
            double Kf = 32767/maxVelocity;
            double Kp = 0.1*Kf;
            double Ki = 0.1*Kp;
            double Kd = 0.1*Ki;
            telemetry.addData("Max Velocity", maxVelocity);
            telemetry.addData("PID", new PIDFCoefficients(Kp,Ki,Kd,Kf));
            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);
            telemetry.addData("Kf", Kf);
            telemetry.update();
        }
    }
    @TeleOp
    public static class MaxVelocityPIDRight extends LinearOpMode {
        DcMotorEx motor;
        double currentVelocity, maxVelocity;
        @Override
        public void runOpMode() throws InterruptedException {
            motor = hardwareMap.get(DcMotorEx.class, "OutakeMotorRight");
            boolean startPID = false;
            maxVelocity = 0;
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            while (opModeInInit()) {
                if (startPID) {
                    if (gamepad1.a) motor.setPower(1);
                    if (gamepad1.b) motor.setPower(0);
                    currentVelocity = motor.getVelocity();
                    if (maxVelocity < currentVelocity) maxVelocity=currentVelocity;
                    telemetry.addData("Max Velocity", maxVelocity);
                    telemetry.addData("Current Velocity", currentVelocity);
                } else {
                    telemetry.addData("BUTTON TO START PID", "Gamepad 1 A");
                    while (!gamepad1.a) idle();
                    startPID = true;
                }
                telemetry.update();
            }
            waitForStart();
            motor.setPower(0);
            double Kf = 32767/maxVelocity;
            double Kp = 0.1*Kf;
            double Ki = 0.1*Kp;
            double Kd = 0.1*Ki;
            telemetry.addData("Max Velocity", maxVelocity);
            telemetry.addData("PID", new PIDFCoefficients(Kp,Ki,Kd,Kf));
            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);
            telemetry.addData("Kf", Kf);
            telemetry.update();
        }
    }
}
