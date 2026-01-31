package org.firstinspires.ftc.teamcode2.pedroPathing;

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

@SuppressWarnings("SpellCheckingInspection")
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(0)
            .forwardZeroPowerAcceleration(-0)
            .lateralZeroPowerAcceleration(-0)
            .translationalPIDFCoefficients(new PIDFCoefficients(0, 0, 0, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0, 0, 0, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0,0,0,0,0))
            .centripetalScaling(0);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static final double VELOCITY_SCALING_FACTOR = 1.2;
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frm")
            .rightRearMotorName("brm")
            .leftRearMotorName("blm")
            .leftFrontMotorName("flm")
            // intake and launcher are "intake" and "launcher". push servo (CR servo) is "push"
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            // intake is reversed
            .xVelocity(0*VELOCITY_SCALING_FACTOR)
            .yVelocity(0*VELOCITY_SCALING_FACTOR);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}