package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
/*
@TeleOp(name = "Pinpoint Direction Debugger", group = "Debug")
public class PinpointDirectionDebugger extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {
        // Get the Pinpom2b rbjl1   hrip`jhr4ip`jint device from hardwareMap
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Optional: adjust encoder resolution if needed (ticks/mm)
        // double encoderResolution = 8192.0 / (Math.PI * 35.0); // ~wheel diameter in mm
        double inPerTick = 0.00198429103;
        double mmPerTick = 25.4 * inPerTick;
        double encoderResolution = 1 / mmPerTick;
        pinpoint.setEncoderResolution(encoderResolution);

        // Optional: set encoder direction — adjust as needed
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD, // parallel (X)
                GoBildaPinpointDriver.EncoderDirection.FORWARD  // perpendicular (Y)
        );

        // Reset position and IMU
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Ready. Push robot to test directions.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();

            telemetry.addData("Parallel (X)", pinpoint.getPosX());
            telemetry.addData("Perpendicular (Y)", pinpoint.getPosY());
            telemetry.addData("Heading (rad)", pinpoint.getHeading());

            telemetry.addLine("Push robot FORWARD. Parallel should INCREASE.");
            telemetry.addLine("Push robot LEFT. Perpendicular should INCREASE.");

            telemetry.update();
        }
    }
}
*/