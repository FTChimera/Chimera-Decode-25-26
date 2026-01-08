package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;




@Autonomous(name = "SlidesTest")
public class SlidesTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            SlidesMovement rightViperSlide = new SlidesMovement(hardwareMap);
            SlidesMovement leftViperSlide = new SlidesMovement(hardwareMap);
            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket telemetryPacket = new TelemetryPacket();

            waitForStart();
            dashboard.sendTelemetryPacket(telemetryPacket);
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .afterTime(0, rightViperSlide.SlidesUp())
                            .afterTime(0, leftViperSlide.SlidesUp())
                            //.afterTime(0, dashboard.sendTelemetryPacket(telemetryPacket))
                            .afterTime(5, rightViperSlide.SlidesDown())
                            .afterTime(5, leftViperSlide.SlidesDown())
                            .lineToX(30)
                            .turn(Math.toRadians(90))
                            .lineToY(30)
                            .strafeTo(new Vector2d(0,-10))
                            .turn(Math.toRadians(90))
                            .lineToX(0)
                            .turn(Math.toRadians(90))
                            .lineToY(0)
                            .turn(Math.toRadians(90))
                            .build()
            );
            dashboard.sendTelemetryPacket(telemetryPacket);



        }
    }
}