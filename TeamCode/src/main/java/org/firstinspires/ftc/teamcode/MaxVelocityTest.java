package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class MaxVelocityTest extends LinearOpMode {
    DcMotorEx rightOutakeMotor;
    DcMotorEx leftOutakeMotor;

    double currentVelocityLeft;
    double currentVelocityRight;
    double maxVelocity = 3000;
    double maxVelocityLeft = 0;
    double maxVelocityRight = 0;

    //maxVelocityLeft=1100
    //maxVelocityRight=1720

    @Override
    public void runOpMode() {
        DcMotorEx rightOutakeMotor = hardwareMap.get(DcMotorEx.class,"OutakeMotorRight");
        DcMotorEx leftOutakeMotor = hardwareMap.get(DcMotorEx.class,"OutakeMotorLeft");
        rightOutakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftOutakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftOutakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOutakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // leftOutakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Kp, Ki, Kd, Kf));
        //rightOutakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Kp, Ki, Kd, Kf));
        waitForStart();
        while (opModeIsActive()) {
            rightOutakeMotor.setPower(1);
            leftOutakeMotor.setPower(1);
            currentVelocityLeft = leftOutakeMotor.getVelocity();
            currentVelocityRight = rightOutakeMotor.getVelocity();

            if (currentVelocityLeft > maxVelocityLeft) {
                maxVelocityLeft = currentVelocityLeft;
            }
            if (currentVelocityRight > maxVelocityRight) {
                maxVelocityRight = currentVelocityRight;
            }

            telemetry.addData("current velocity Left", currentVelocityLeft);
            telemetry.addData("current velocity Right", currentVelocityRight);
            telemetry.addData("maximum velocity Left", maxVelocityLeft);
            telemetry.addData("maximum velocity Right", maxVelocityRight);
            telemetry.update();
        }
    }
}


