package org.firstinspires.ftc.teamcode2.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode2.pedroPathing.Constants.driveConstants;

@SuppressWarnings("SpellCheckingInspection")
public class TeleOpDriveControl {
    public DcMotor lf, rf, lb, rb;

    public TeleOpDriveControl(HardwareMap hwMap) {
        this.lf = hwMap.get(DcMotor.class, driveConstants.leftFrontMotorName);
        this.rf = hwMap.get(DcMotor.class, driveConstants.rightFrontMotorName);
        this.lb = hwMap.get(DcMotor.class, driveConstants.leftRearMotorName);
        this.rb = hwMap.get(DcMotor.class, driveConstants.rightRearMotorName);

        lf.setDirection(driveConstants.leftFrontMotorDirection);
        rf.setDirection(driveConstants.rightFrontMotorDirection);
        lb.setDirection(driveConstants.leftRearMotorDirection);
        rb.setDirection(driveConstants.rightRearMotorDirection);
    }


    public void move(Gamepad gamepad, double scalar) {
        double y = -gamepad.left_stick_y; // Forward/Backward
        double x = gamepad.left_stick_x*1.1;  // Left/Right counteract imperfect strafing
        double rx = gamepad.right_stick_x; // Rotation

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1) / scalar;
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
