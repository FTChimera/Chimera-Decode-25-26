package org.firstinspires.ftc.teamcode2.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode2.Systems.Constants.pedroMecanumDriveConstants;

@SuppressWarnings("SpellCheckingInspection")
public class TeleOpDriveControl {
    public DcMotor lf, rf, lb, rb;

    public TeleOpDriveControl(HardwareMap hwMap) {
        this.lf = hwMap.get(DcMotor.class, pedroMecanumDriveConstants.leftFrontMotorName);
        this.rf = hwMap.get(DcMotor.class, pedroMecanumDriveConstants.rightFrontMotorName);
        this.lb = hwMap.get(DcMotor.class, pedroMecanumDriveConstants.leftRearMotorName);
        this.rb = hwMap.get(DcMotor.class, pedroMecanumDriveConstants.rightRearMotorName);

        lf.setDirection(pedroMecanumDriveConstants.leftFrontMotorDirection);
        rf.setDirection(pedroMecanumDriveConstants.rightFrontMotorDirection);
        lb.setDirection(pedroMecanumDriveConstants.leftRearMotorDirection);
        rb.setDirection(pedroMecanumDriveConstants.rightRearMotorDirection);
    }


    public void move(Gamepad gamepad) {
        // applyPolynomialToDriveInputs assumes pedro drive - so we negate some of them.
        double y = Constants.applyPolynomialToDriveInputs(gamepad.left_stick_y); // Forward/Backward
        double x = -Constants.applyPolynomialToDriveInputs(gamepad.left_stick_x*1.1);  // Left/Right counteract imperfect strafing
        double rx = -Constants.applyPolynomialToDriveInputs(gamepad.right_stick_x); // Rotation

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        lf.setPower(frontLeftPower);
        lb.setPower(backLeftPower);
        rf.setPower(frontRightPower);
        rb.setPower(backRightPower);
    }
}
