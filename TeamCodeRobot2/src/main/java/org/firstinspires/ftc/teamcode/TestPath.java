package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "TestPath")
public class TestPath extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();


            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .lineToX(50)
                            .strafeTo(new Vector2d(0,0))
                            .lineToX(50)
                            .strafeTo(new Vector2d(0,0))
                            .lineToX(50)
                            .strafeTo(new Vector2d(0,0))
                            .lineToX(50)
                           .strafeTo(new Vector2d(0,0))
                            .lineToX(50)
                           .strafeTo(new Vector2d(0,0))
                           .lineToX(50)
                            .strafeTo(new Vector2d(0,0))
                            .turn(Math.toRadians(-90))
                            .build()
            );
           // Actions.runBlocking(
              //      drive.actionBuilder(beginPose)
                         //   .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            //.splineTo(new Vector2d(0, 60), Math.PI)
                            //.build()
           // );

        }
    }
}