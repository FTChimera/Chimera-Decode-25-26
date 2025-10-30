package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ChimeraTeleOpPedro;
import org.firstinspires.ftc.teamcode.Vision.LLToolkit;

@TeleOp
public class LimelightObjectDetection extends LinearOpMode {
    enum AllianceColor {BLUE, RED};
    AllianceColor allianceColor;
    public double targetX = 0;
    public double targetY = 0;
    public void runOpMode() {
        allianceColor = AllianceColor.RED; // TEST RED ALLIANCE
        int AprilID = allianceColor==AllianceColor.RED ? 24 : 20;
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LLToolkit.ChimeraLL limelight = new LLToolkit.ChimeraLL();
        Limelight3A lldevice = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setDevice(lldevice);
        waitForStart();
        limelight.startLLWithPipeline(0);
        while (opModeIsActive()) {
            limelight.LLUpdate();double[]result=limelight.getLatest();
            double tx = result[0];double ty = result[1];double ta = result[2];double tid = result[3];double dist = result[4];double trx = 0; // trx rotation not implemented yet
            double denominator = Math.max(Math.abs(ty) + Math.abs(tx) + Math.abs(trx), 1);
            double frontLeftPower = 0.85 *(ty + tx + trx) / denominator;
            double backLeftPower = 0.85 *(ty - tx + trx) / denominator;
            double frontRightPower = 0.85 *(ty - tx - trx) / denominator;
            double backRightPower = 0.85* (ty + tx - trx) / denominator;
            if (gamepad2.a) {frontLeftMotor.setPower(frontLeftPower);backLeftMotor.setPower(backLeftPower);frontRightMotor.setPower(frontRightPower);backRightMotor.setPower(backRightPower);}
        }
    }
}