package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//TODO- Change the robots mass(in kg)
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5.2)
            .forwardZeroPowerAcceleration(-43.86484666397469)
            .lateralZeroPowerAcceleration(-81.63848969598853)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1.1, 0, 0.01, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0,0.0001,0.6,0.003))
            .centripetalScaling(0.0009);









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
            .forwardPodY(6)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(59.25861989419292)
            .yVelocity(49.53692626953125);






          //  .lateralZeroPowerAcceleration(0.9);
}

//-48.08076261884704

// TODO - Delete this later. This is copied to check if the names within the quote
//leftFront = hardwareMap.get(DcMotorEx .class, "frontLeftMotor");
//leftBack = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
//rightBack = hardwareMap.get(DcMotorEx.class, "backRightMotor");
//rightFront = hardwareMap.get(DcMotorEx.class, "frontRightMotor");


