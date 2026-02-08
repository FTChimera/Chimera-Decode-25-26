package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import static org.firstinspires.ftc.teamcode.NewBot.Constants.applyPolynomialToDriveInputs;
import static org.firstinspires.ftc.teamcode.NewBot.Constants.pedroMecanumDriveConstants;

@SuppressWarnings("SpellCheckingInspection")
public class SimpleTeleOpDrive {

    public DcMotor lf, rf, lb, rb;
    private IMU imu;

    public SimpleTeleOpDrive(HardwareMap hwMap) {

        lf = hwMap.get(DcMotor.class, pedroMecanumDriveConstants.leftFrontMotorName);
        rf = hwMap.get(DcMotor.class, pedroMecanumDriveConstants.rightFrontMotorName);
        lb = hwMap.get(DcMotor.class, pedroMecanumDriveConstants.leftRearMotorName);
        rb = hwMap.get(DcMotor.class, pedroMecanumDriveConstants.rightRearMotorName);

        lf.setDirection(pedroMecanumDriveConstants.leftFrontMotorDirection);
        rf.setDirection(pedroMecanumDriveConstants.rightFrontMotorDirection);
        lb.setDirection(pedroMecanumDriveConstants.leftRearMotorDirection);
        rb.setDirection(pedroMecanumDriveConstants.rightRearMotorDirection);

        // --- IMU SETUP ---
        imu = hwMap.get(IMU.class, "imu");

        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(params);
        imu.resetYaw();
    }

    /** Call this when driver presses "reset heading" */
    public void resetHeading() {
        imu.resetYaw();
    }

    public void drive(Gamepad gamepad, boolean robotRelative) {

        // Raw inputs
        double y = applyPolynomialToDriveInputs(gamepad.left_stick_y); // forward
        double x = -applyPolynomialToDriveInputs(gamepad.left_stick_x * 1.1); // strafe
        double rx = -applyPolynomialToDriveInputs(gamepad.right_stick_x); // rotate

        // --- FIELD CENTRIC ---
        if (!robotRelative) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(); // radians

            double cosA = Math.cos(-heading);
            double sinA = Math.sin(-heading);

            double fieldX = x * cosA - y * sinA;
            double fieldY = x * sinA + y * cosA;

            x = fieldX;
            y = fieldY;
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double frontLeftPower  = (y + x + rx) / denominator;
        double backLeftPower   = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower  = (y + x - rx) / denominator;

        lf.setPower(frontLeftPower);
        lb.setPower(backLeftPower);
        rf.setPower(frontRightPower);
        rb.setPower(backRightPower);
    }
}