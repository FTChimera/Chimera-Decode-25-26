package org.firstinspires.ftc.teamcode.NewBot;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//TODO- Change the robots mass(in kg)
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.4)
            .forwardZeroPowerAcceleration(-36.816499666421194)
            .lateralZeroPowerAcceleration(-61.537050148361466)
            .translationalPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(0.067, 0, 0.05, 0.01))
            .headingPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(0.9, 0, 0.08, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0,0.0001,3.5,0.00001))
            .centripetalScaling(0.00003);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    // TODO- Change the X and Y pod offsets

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-0.3125)
            .strafePodX(-3.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);



    public static final double VELOCITY_SCALING_FACTOR = 1;
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
            // intake and launcher are "intake" and "launcher". transfer is "transfer"
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            // intake is reversed, transfer is reversed
            .xVelocity(85.22694408987452*VELOCITY_SCALING_FACTOR)
            .yVelocity(74.93444535863681*VELOCITY_SCALING_FACTOR);


    //  .lateralZeroPowerAcceleration(0.9);
}

//-48.08076261884704

// TODO - Delete this later. This is copied to check if the names within the quote
//leftFront = hardwareMap.get(DcMotorEx .class, "frontLeftMotor");
//leftBack = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
//rightBack = hardwareMap.get(DcMotorEx.class, "backRightMotor");
//rightFront = hardwareMap.get(DcMotorEx.class, "frontRightMotor");


