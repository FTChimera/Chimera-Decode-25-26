package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Systems.Consts;

@TeleOp(name = "PID Tuning", group = "Tuning")
public class PIDTuning extends OpMode {

    SimpleTeleOpDrive teleOpDrive;

    double leftP = 0.0, leftI = 0.0, leftD = 0.0, leftF = 0.0;
    double rightP = 0.0, rightI = 0.0, rightD = 0.0, rightF = 0.0;

    double maxLeftVelocity = 0;
    double maxRightVelocity = 0;

    // Velocity testing
    double targetVelocity = 0;

    enum Coefficient {
        LEFT_P, LEFT_I, LEFT_D, LEFT_F,
        RIGHT_P, RIGHT_I, RIGHT_D, RIGHT_F
    }

    Coefficient currentCoeff = Coefficient.LEFT_P;

    boolean lastUp, lastDown;
    boolean lastA, lastB, lastX, lastY;

    @Override
    public void init() {
        teleOpDrive = new SimpleTeleOpDrive(hardwareMap);

        leftP = Consts.leftPIDF.p;
        leftI = Consts.leftPIDF.i;
        leftD = Consts.leftPIDF.d;
        leftF = Consts.leftPIDF.f;

        rightP = Consts.rightPIDF.p;
        rightI = Consts.rightPIDF.i;
        rightD = Consts.rightPIDF.d;
        rightF = Consts.rightPIDF.f;

        teleOpDrive.LeftOutake.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        teleOpDrive.RightOutake.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("PID Tuning Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Drive normally
        teleOpDrive.MoveDriveTrain(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        handleCoefficientSelection();
        handlePIDAdjustment();
        handleVelocityAdjustment();

        // Apply PIDF dynamically
        PIDFCoefficients leftPID =
                new PIDFCoefficients(leftP, leftI, leftD, leftF);
        PIDFCoefficients rightPID =
                new PIDFCoefficients(rightP, rightI, rightD, rightF);

        teleOpDrive.updatePIDFCoefficients(leftPID, rightPID);

        maxLeftVelocity = Math.max(
                maxLeftVelocity,
                teleOpDrive.LeftOutake.getVelocity()
        );
        maxRightVelocity = Math.max(
                maxRightVelocity,
                teleOpDrive.RightOutake.getVelocity()
        );

        telemetryOutput();
    }

    private void handleCoefficientSelection() {
        if (gamepad1.a && !lastA) currentCoeff = Coefficient.LEFT_P;
        if (gamepad1.b && !lastB) currentCoeff = Coefficient.LEFT_I;
        if (gamepad1.x && !lastX) currentCoeff = Coefficient.LEFT_D;
        if (gamepad1.y && !lastY) currentCoeff = Coefficient.LEFT_F;

        if (gamepad1.dpad_left) currentCoeff = Coefficient.RIGHT_P;
        if (gamepad1.dpad_right) currentCoeff = Coefficient.RIGHT_I;
        if (gamepad1.left_stick_button) currentCoeff = Coefficient.RIGHT_D;
        if (gamepad1.right_stick_button) currentCoeff = Coefficient.RIGHT_F;

        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
    }

    private void handlePIDAdjustment() {
        double step = gamepad1.left_bumper ? 0.0001 :
                gamepad1.right_bumper ? 0.01 : 0.001;

        if (gamepad1.dpad_up && !lastUp) {
            adjustCurrent(step);
        }
        if (gamepad1.dpad_down && !lastDown) {
            adjustCurrent(-step);
        }

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
    }

    private void adjustCurrent(double amount) {
        switch (currentCoeff) {
            case LEFT_P: leftP += amount; break;
            case LEFT_I: leftI += amount; break;
            case LEFT_D: leftD += amount; break;
            case LEFT_F: leftF += amount; break;
            case RIGHT_P: rightP += amount; break;
            case RIGHT_I: rightI += amount; break;
            case RIGHT_D: rightD += amount; break;
            case RIGHT_F: rightF += amount; break;
        }
    }

    private void handleVelocityAdjustment() {
        targetVelocity += (gamepad1.right_trigger - gamepad1.left_trigger) * 50;
        targetVelocity = Math.max(0, targetVelocity);

        teleOpDrive.SetOutakeVelocity(targetVelocity);
    }

    private void telemetryOutput() {
        telemetry.addLine("==== PID TUNING ====");
        telemetry.addData("Left P I D F",
                "%.4f %.4f %.4f %.4f",
                leftP, leftI, leftD, leftF);
        telemetry.addData("Right P I D F",
                "%.4f %.4f %.4f %.4f",
                rightP, rightI, rightD, rightF);
        telemetry.addData("Editing", currentCoeff);

        telemetry.addLine("==== VELOCITY ====");
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Left Velocity", teleOpDrive.LeftOutake.getVelocity());
        telemetry.addData("Right Velocity", teleOpDrive.RightOutake.getVelocity());

        telemetry.addData("Max Left Velocity", maxLeftVelocity);
        telemetry.addData("Max Right Velocity", maxRightVelocity);

        telemetry.update();
    }
}