package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

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
        Limelight3A limelight1 = hardwareMap.get(Limelight3A.class, "limelight");
        limelight1.pipelineSwitch(1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int curPipe = 1;
        double FrameCheck = 0;
        boolean isRed = gamepad1.a; // fixed at init
        boolean LocalizationOn;
        double tx = 0, ty = 0, ta = 0;
        Pose3D botpose = null;
        waitForStart();
        limelight1.start();

        while (opModeIsActive()) {
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