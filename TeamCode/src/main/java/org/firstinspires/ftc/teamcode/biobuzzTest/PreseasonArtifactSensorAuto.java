package org.firstinspires.ftc.teamcode.biobuzzTest;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.NewBot.Constants;



@TeleOp(name = "Preseason Artifact Sensor Auto", group = "Test")
public class PreseasonArtifactSensorAuto extends LinearOpMode {
    // CONSTANTS

    PIDFCoefficients forwardPIDF = new PIDFCoefficients(0.9, 0, 0, 0);
    PIDFCoefficients strafePIDF = new PIDFCoefficients(0, 0, 0, 0);
    PIDFCoefficients headingPIDF = new PIDFCoefficients(0.001, 0, 0, 0);

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Follower follower = Constants.createPedroFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        limelight.pipelineSwitch(1); // Color detection pipeline
        PIDFController<Pose> controller = getController();
        waitForStart();
        limelight.start();
        follower.startTeleopDrive();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                // Detect Artifact
                try {
                    LLResultTypes.ColorResult colorResult = result.getColorResults().get(0);
                    double tx = colorResult.getTargetXPixels();
                    double area = Range.clip(colorResult.getTargetArea(), 2, 70);
                    double distanceProportionalApprox = 1 / Math.sqrt(area); // Almost Proportional to distance
                    controller.setTarget(new Pose(0, -distanceProportionalApprox, tx));
                    controller.update(new Pose());
                    Pose output = controller.getOutput();
                    telemetry.addData("Controller Output", "x: %.2f, y: %.2f, heading: %.2f", output.getX(), output.getY(), output.getHeading());
                    follower.setTeleOpDrive(
                            -output.getY(),
                            -output.getX(),
                            -output.getHeading(),
                            true
                    );
                    telemetry.addData("Artifact Detection", "Artifact detected at tx: %.2f, area: %.2f", tx, area);
                } catch (Exception e) {
                    if (e instanceof IndexOutOfBoundsException) {
                        telemetry.addData("Artifact Detection", "No artifact detected");
                    } else {
                        telemetry.addData("Artifact Detection", "Error: " + e.getMessage());
                    }
                    controller.reset();
                }
            }

            // MANUAL CONTROL
            if (gamepad1.backWasPressed()) {
                controller.reset();
            }

            if (gamepad1.a) {
                follower.setTeleOpDrive(0,0,0, true);
            }

            follower.update();
            telemetry.update();
        }
        limelight.stop();
    }

    org.firstinspires.ftc.teamcode.biobuzzTest.PIDFCoefficients getPIDFCoeff() {
        return new org.firstinspires.ftc.teamcode.biobuzzTest.PIDFCoefficients(
                new double[]{strafePIDF.P, forwardPIDF.P, headingPIDF.P},
                new double[]{strafePIDF.I, forwardPIDF.I, headingPIDF.I},
                new double[]{strafePIDF.D, forwardPIDF.D, headingPIDF.D},
                new double[]{strafePIDF.F, forwardPIDF.F, headingPIDF.F},
                new double[]{0.5, 0.5, 0.5}, // Tolerance
                new double[]{1.0, 1.0, 1.0} // Integral limit
        );
    }
    PIDFController<Pose> getController() {
        PIDFController<Pose> controller = new PIDFController<>(getPIDFCoeff());
        controller.setStateSupplier(pose -> new double[]{pose.getX(), pose.getY(), pose.getHeading()});
        controller.setInverseStateSupplier(array -> new Pose(array[0], array[1], array[2]));
        return controller;
    }
}
