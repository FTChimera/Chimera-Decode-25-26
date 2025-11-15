package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TestOuttakeNew extends LinearOpMode {
    boolean outtakemotor;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor outtakeMotor = hardwareMap.get(DcMotor.class, "outtakeNew");
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if (gamepad1.a) {
                outtakemotor = !outtakemotor;
            }
            outtakeMotor.setPower(outtakemotor?0.8:0.0);
        }
    }
}
