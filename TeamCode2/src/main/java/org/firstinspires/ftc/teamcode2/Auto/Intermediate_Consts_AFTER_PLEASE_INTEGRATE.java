package org.firstinspires.ftc.teamcode2.Auto;

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

public class Intermediate_Consts_AFTER_PLEASE_INTEGRATE {
    public static FollowerConstants pedroFollowerConstants = new FollowerConstants()
            .mass(11.4)
            .forwardZeroPowerAcceleration(42.04281447909285)
            .lateralZeroPowerAcceleration(-67.09362398540848)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.035, 0, 0.001, 0.023))
            .headingPIDFCoefficients(new PIDFCoefficients(0.99, 0, 0.03, 0.0089))
            ;
    public static PathConstraints pedroPathConstraints = new PathConstraints(0.99, 100, 1, 1.2);
    public static PinpointConstants pedroLocalizerConstants = new PinpointConstants()
            .forwardPodY(-0.3125)
            .strafePodX(-3.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static MecanumConstants pedroMecanumDriveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frm")
            .rightRearMotorName("brm")
            .leftRearMotorName("blm")
            .leftFrontMotorName("flm")
            // intake and launcher are "intake" and "launcher". transfer is "transfer"
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            // intake is reversed, transfer is reversed
            .xVelocity(86.16603580985483)
            .yVelocity(69.43380328801673)
            ;

    public static Follower createPedroFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(pedroFollowerConstants, hardwareMap)
                .pathConstraints(pedroPathConstraints)
                .mecanumDrivetrain(pedroMecanumDriveConstants)
                .pinpointLocalizer(pedroLocalizerConstants)
                .build();
    }
}
