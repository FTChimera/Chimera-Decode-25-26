package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Systems.Consts.AUTO_RED_STARTING_POSE;
import static org.firstinspires.ftc.teamcode.Systems.Consts.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.Systems.Consts.RED_GOAL;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Consts;
import org.firstinspires.ftc.teamcode.Vision.aprilTagIndicatorTest;
import org.firstinspires.ftc.teamcode.pedroAuto.Constants;
import org.firstinspires.ftc.teamcode.pedroAuto.Tuning;

@Configurable
@TeleOp(name="Tester Selectable")
public class Tester extends SelectableOpMode {
    public Tester() {
        super("Select an OpMode", s -> {
            s.folder("Launchers Test", l -> {
                l.add("Max Velocity PID Test Tuner", MaxVelocityTest::new);
            });
            s.folder("Limelight and RGB Indicator", l -> {
                l.add("April Tag Indicator Test", aprilTagIndicatorTest::new);
            });
            s.folder("Pedro Pathing", l -> {
                l.add("Pedro Velocity Values", TestVelocityValues::new);
                l.add("Pedro Tuning", Tuning::new);
            });
        });
    }
    public static class TestVelocityValues extends LinearOpMode {
        SimpleTeleOpDrive teleOpDrive;
        double OutakeVelocity;
        private Follower follower;
        private Pose goalPose() {
            double heading = follower.getHeading();

            // Normalize heading to [0, 2π)
            while (heading < 0) heading += 2 * Math.PI;
            while (heading >= 2 * Math.PI) heading -= 2 * Math.PI;

            // Facing generally "up" the field (toward red side)
            if (heading > Math.PI / 2 && heading < 3 * Math.PI / 2) {
                return RED_GOAL;
            }
            return BLUE_GOAL;
        }
        private double distance() {
            Pose curPose = follower.getPose();
            double dx = goalPose().getX() - curPose.getX();
            double dy = goalPose().getY() - curPose.getY();
            double distance = Math.hypot(dx, dy);
            return distance;
        }
        @Override
        public void runOpMode() throws InterruptedException {
            sleep(2000);
            teleOpDrive = new SimpleTeleOpDrive(hardwareMap);
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(AUTO_RED_STARTING_POSE);
            waitForStart();

            while (opModeIsActive()) {
                follower.update();
                teleOpDrive.MoveDriveTrain(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
                teleOpDrive.Intake.setPower(gamepad1.x?1:0);
                if (gamepad1.aWasPressed()) {
                    teleOpDrive.SetOutakeVelocity(Consts.TARGET_VELOCITY_BACK_LAUNCH_ZONE);
                    OutakeVelocity = Consts.TARGET_VELOCITY_BACK_LAUNCH_ZONE;
                }
                if (gamepad1.yWasPressed()) {
                    teleOpDrive.SetOutakeVelocity(Consts.TARGET_VELOCITY_FRONT_LAUNCH_ZONE);
                    OutakeVelocity = Consts.TARGET_VELOCITY_FRONT_LAUNCH_ZONE;
                }
                if (gamepad1.bWasReleased()) {
                    teleOpDrive.SetOutakeVelocity(0);
                    OutakeVelocity=0;
                }
                if (gamepad1.dpadUpWasPressed()) {teleOpDrive.push(OutakeVelocity-100);}
                if (gamepad1.rightBumperWasPressed()) {
                    OutakeVelocity+=50;
                    teleOpDrive.SetOutakeVelocity(OutakeVelocity);
                }
                if (gamepad1.leftBumperWasPressed()) {
                    OutakeVelocity-=50;
                    teleOpDrive.SetOutakeVelocity(OutakeVelocity);
                }
                telemetry.addData("Outake Velocity Set To:", OutakeVelocity);
                telemetry.addData("Left Outake Velocity", teleOpDrive.LeftOutake.getVelocity());
                telemetry.addData("Right Outake Velocity", teleOpDrive.RightOutake.getVelocity());
                telemetry.addLine("");
                telemetry.addData("Pose X", follower.getPose().getX());
                telemetry.addData("Pose Y", follower.getPose().getY());
                telemetry.addData("Pose Heading", follower.getHeading());
                telemetry.addLine("");
                telemetry.addLine("");
                telemetry.addLine("VELOCITY TABLE ENTRY:");
                telemetry.addLine("");
                telemetry.addLine(distance() + ", " + OutakeVelocity);
                telemetry.update();
            }
        }
    }
}
