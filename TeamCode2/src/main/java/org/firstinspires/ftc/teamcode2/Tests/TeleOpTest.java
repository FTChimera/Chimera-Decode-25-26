package org.firstinspires.ftc.teamcode2.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode2.Systems.Constants;
import org.firstinspires.ftc.teamcode2.Systems.TeleOpDriveControl;

@SuppressWarnings("SpellCheckingInspection")
@TeleOp(name="TeleOpTest", group="Tests")
public class TeleOpTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TeleOpDriveControl driveControl = new TeleOpDriveControl(hardwareMap);
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        //CRServo servo = hardwareMap.crservo.get("push");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.LaunchPIDF);
        waitForStart();
        while (opModeIsActive()) {
            driveControl.move(gamepad1);

            double intakeMove =
                    gamepad1.right_trigger - gamepad1.left_trigger;
            intake.setPower(intakeMove);

            if (gamepad1.a) {
                launcher.setVelocity(Constants.TARGET_VELOCITY_BACK_LAUNCH_ZONE);
            }
            if (gamepad1.b) {
                launcher.setVelocity(Constants.STOP_VELOCITY);
            }
            if (gamepad1.y) {
                launcher.setVelocity(Constants.TARGET_VELOCITY_FRONT_LAUNCH_ZONE);
            }

            if (gamepad1.left_bumper) {
               // servo.setPower(Consts.TRANSFER_UP_POSITION);
            } else {
                //servo.setPower(Consts.TRANSFER_DOWN_POSITION);
            }
        }
    }

}
