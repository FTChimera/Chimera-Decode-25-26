package org.firstinspires.ftc.teamcode2.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode2.Systems.TeleOpDriveControl;

@TeleOp
public class DriveAndIntake extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TeleOpDriveControl driveControl = new TeleOpDriveControl(hardwareMap);
        DcMotor intake = hardwareMap.dcMotor.get("intake");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            driveControl.move(
                    gamepad1, 0.7
            );

            double intakeMove =
                    gamepad1.left_trigger - gamepad1.right_trigger;
            intake.setPower(intakeMove);
        }
    }

}
