package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp
public class LimelightObjectDetection extends LinearOpMode {

    @Override
    public void runOpMode() {
        Limelight3A limelight1 = hardwareMap.get(Limelight3A.class, "limelight"); // "limelight" should match the configured name on the Driver Hub
        limelight1.pipelineSwitch(0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight1.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx(); // Horizontal offset from crosshair to target center
                double ty = result.getTy(); // Vertical offset from crosshair to target center
                double ta = result.getTa(); // Target area (0-100% of image)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Status", "No valid target found");
            }
            telemetry.update();
        }
    }
}
