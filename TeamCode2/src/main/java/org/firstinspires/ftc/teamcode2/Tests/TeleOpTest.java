package org.firstinspires.ftc.teamcode2.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode2.Systems.Consts;
import org.firstinspires.ftc.teamcode2.Systems.TeleOpDriveControl;
import org.firstinspires.ftc.teamcode2.pedroPathing.Constants;

@SuppressWarnings("SpellCheckingInspection")
@TeleOp
public class TeleOpTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TeleOpDriveControl driveControl = new TeleOpDriveControl(hardwareMap);
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        CRServo servo = hardwareMap.crservo.get("push");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Consts.LaunchPIDF);
        waitForStart();
        while (opModeIsActive()) {
            driveControl.move(gamepad1);

            double intakeMove =
                    gamepad1.right_trigger - gamepad1.left_trigger;
            intake.setPower(intakeMove);

            if (gamepad1.a) {
                launcher.setVelocity(Consts.TARGET_VELOCITY_BACK_LAUNCH_ZONE);
            }
            if (gamepad1.b) {
                launcher.setVelocity(Consts.STOP_VELOCITY);
            }
            if (gamepad1.y) {
                launcher.setVelocity(Consts.TARGET_VELOCITY_FRONT_LAUNCH_ZONE);
            }

            if (gamepad1.left_bumper) {
                servo.setPower(Consts.SERVO_UP_POSITION);
            } else {
                servo.setPower(Consts.SERVO_DOWN_POSITION);
            }
        }
    }

}
