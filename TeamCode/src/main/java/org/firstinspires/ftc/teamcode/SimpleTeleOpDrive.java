package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Systems.Consts;

public class SimpleTeleOpDrive {
    public DcMotor frontLeft,frontRight,backLeft,backRight,Intake;
    public DcMotorEx LeftOutake,RightOutake;

    HardwareMap hardwareMap;

    public SimpleTeleOpDrive(HardwareMap hwMap) {
        hardwareMap=hwMap;
        frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        backLeft = hardwareMap.dcMotor.get("backLeftMotor");
        backRight = hardwareMap.dcMotor.get("backRightMotor");

        Intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        RightOutake = hardwareMap.get(DcMotorEx.class, "OutakeMotorRight");
        LeftOutake = hardwareMap.get(DcMotorEx.class, "OutakeMotorLeft");

        frontLeft.setDirection(FORWARD);
        frontRight.setDirection(FORWARD);
        backLeft.setDirection(REVERSE);
        backRight.setDirection(REVERSE);
        Intake.setDirection(REVERSE);
        LeftOutake.setDirection(FORWARD);
        RightOutake.setDirection(REVERSE);

        LeftOutake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Consts.leftPIDF);
        RightOutake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Consts.rightPIDF);
    }

    public void MoveDriveTrain(double gamepady, double gamepadx, double gamepadrx) {
        double y = -gamepady;
        double x = gamepadx * 1.1;
        double rx = gamepadrx;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = 1 *(y + x + rx) / denominator;
        double backLeftPower = 1 *(y - x + rx) / denominator;
        double frontRightPower = 1 *(y - x - rx) / denominator;
        double backRightPower = 1 * (y + x - rx) / denominator;
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void SetOutakeVelocity(double velocity) {
        LeftOutake.setVelocity(velocity);
        RightOutake.setVelocity(velocity);
    }
}
