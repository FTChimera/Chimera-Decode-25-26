package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Systems.Consts;
import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode.Systems.RGBIndicator;

@TeleOp
public class aprilTagIndicatorTest extends LinearOpMode {
    Consts.AllianceColor allianceColor;
    LimelightSystem.ChimeraLL limelight = new LimelightSystem.ChimeraLL();
    Servo rgbIndicator;


    public void runOpMode() {
        allianceColor = Consts.AllianceColor.RED; // TEST
        telemetry.addData("ALLIANCE COLOR", allianceColor);telemetry.update();
        rgbIndicator = hardwareMap.get(Servo.class, "rgb");
        limelight.setDevice(hardwareMap.get(Limelight3A.class, "limelight"));

        waitForStart();
        limelight.startLLWithPipeline(allianceColor==Consts.AllianceColor.RED?4:5);

        while (opModeIsActive()) {
            limelight.LLUpdate();
            telemetry.addData("LLScore()", limelight.getLLScore());
            telemetry.addData("Tx",limelight.tx);
            telemetry.addData("Dist",limelight.dist);

            if (limelight.getLLScore() < 5) {
                // GREEN
                rgbIndicator.setPosition(RGBIndicator.GREEN_PWM);
            } else if (limelight.getLLScore() < 10) {
                // ORANGE
                rgbIndicator.setPosition(RGBIndicator.ORANGE_PWM);
            } else {
                // OFF
                rgbIndicator.setPosition(RGBIndicator.BLACK_PWM);
            }
        }
    }
}
