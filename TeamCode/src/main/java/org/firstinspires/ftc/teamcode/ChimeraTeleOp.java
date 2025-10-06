package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ChimeraTeleOp", group = "AbsolutePriority")// Name and Group
public class ChimeraTeleOp extends LinearOpMode {

    int slidePosition = 0;
    int MAX_SLIDE_POSITION = 1400;
    int SLIDE_POSITION_INCREASE_AMOUNT = 30;

    DcMotor rightViperSlide = null;
    DcMotor leftViperSlide = null;


    void moveSlides(int slidePosition) {
        rightViperSlide.setTargetPosition(slidePosition);
        rightViperSlide.setPower(1);
        leftViperSlide.setTargetPosition(slidePosition);
        leftViperSlide.setPower(1);
        leftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor OutakeMotorRight = hardwareMap.dcMotor.get("OutakeMotorRight");
        DcMotor OutakeMotorLeft = hardwareMap.dcMotor.get("OutakeMotorLeft");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

      //  rightViperSlide = hardwareMap.dcMotor.get("rightViperSlide");
        //leftViperSlide = hardwareMap.dcMotor.get("leftViperSlide");

      //  leftViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftViperSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightViperSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        OutakeMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        OutakeMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
     //   leftViperSlide.setDirection(DcMotorSimple.Direction.REVERSE);
       // rightViperSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = 0.85 *(y + x + rx) / denominator;
            double backLeftPower = 0.85 *(y - x + rx) / denominator;
            double frontRightPower = 0.85 *(y - x - rx) / denominator;
            double backRightPower = 0.85* (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (gamepad1.dpad_up && rightViperSlide.getCurrentPosition() <= MAX_SLIDE_POSITION) {
                slidePosition += 30;
            } else if (gamepad1.dpad_down) {
                slidePosition -= 30;
            }
            if (slidePosition < 0) {
                slidePosition = 0;
            }
            if (slidePosition != 0) {
                moveSlides(slidePosition);
            }

            if (gamepad1.a){
                OutakeMotorLeft.setPower(1);
                OutakeMotorRight.setPower(1);
            }

            if (gamepad1.b){
                OutakeMotorRight.setPower(0);
                OutakeMotorLeft.setPower(0);
            }
            if(gamepad1.x){
                intakeMotor.setPower(1);
            }
            if(gamepad1.y){
                intakeMotor.setPower(0);
            }

           // telemetry.addData("Right Slide Position", rightViperSlide.getCurrentPosition());
           // telemetry.addData("Left Slide Position", leftViperSlide.getCurrentPosition());
           // telemetry.addData("Slide Increment", slidePosition);

            telemetry.update();
        }
    }
}

