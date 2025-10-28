package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/*
Pipelines:
0: TEST
1: Purple
2: green
3: obelisk detection
4: red
5: blue
 */
@TeleOp
public class LimelightObjectDetection extends LinearOpMode {

    enum AllianceColor {
        BLUE,
        RED
    };
    AllianceColor allianceColor = AllianceColor.RED;
    int goalID = 24; // CHANGE FOR ALLIANCE COLOR - Blue: 20, Red: 24
    public int changePipeline(int curPipe, boolean isRed) {
        switch (curPipe) {
            case 1:
                return 2; // purple → green
            case 2:
                return 3; // green → obelisk
            case 3:
                return isRed ? 4 : 5; // obelisk → red or blue
            case 4:
                return 1;
            case 5:
                return 1; // back to purple
            default:
                return 1; // start at purple
        }
    }

    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        Limelight3A limelight1 = hardwareMap.get(Limelight3A.class, "limelight");
        limelight1.pipelineSwitch(1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int curPipe = 1;
        double FrameCheck = 0;
        boolean isRed = gamepad1.a; // fixed at init
        boolean LocalizationOn;
        double tx = 0, ty = 0, ta = 0, tid = 0;
        Pose3D botpose = null;
        waitForStart();
        limelight1.start();

        while (opModeIsActive()) {
                if (gamepad2.a && tid==goalID) {
                double denominator = Math.max(Math.abs(tx), 1);
                double frontLeftPower = 0.85 * (tx) / denominator;
                double backLeftPower = 0.85 * (-tx) / denominator;
                double frontRightPower = 0.85 * (-tx) / denominator;
                double backRightPower = 0.85 * (tx) / denominator;
                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Current Pipeline", curPipe);
            curPipe = changePipeline(curPipe, isRed);
            limelight1.pipelineSwitch(curPipe);

            LocalizationOn = (curPipe == 4) || (curPipe == 5);

            LLResult result = limelight1.getLatestResult();

            if (result != null && result.isValid()) {
                FrameCheck = 5000;

                tx = result.getTx();
                ty = result.getTy();
                ta = result.getTa();

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    tid = fiducial.getFiducialId();
                }



                if (LocalizationOn) {
                    botpose = result.getBotpose();
                    if (botpose != null) {
                        telemetry.addData("Bot Position", botpose.getPosition()); // ftc - external.navigation.position
                    }
                }
            } else {
                FrameCheck--;
            }

            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("ta", ta);
            telemetry.update();
        }

        telemetry.addData("Status", "Stopped");
        telemetry.update();
        limelight1.stop();
    }
}