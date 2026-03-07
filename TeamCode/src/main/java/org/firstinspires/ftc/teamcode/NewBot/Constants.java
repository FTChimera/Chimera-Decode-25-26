package org.firstinspires.ftc.teamcode.NewBot;

import static org.firstinspires.ftc.teamcode.NewBot.Constants.AllianceColor.BLUE;
import static org.firstinspires.ftc.teamcode.NewBot.Constants.AllianceColor.RED;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@SuppressWarnings("SpellCheckingInspection")
public class Constants {


    public static double applyPolynomialToDriveInputs(double input) {
        double output = input;
        output = output*Math.abs(output); // square for more control
        output = output*-0.8; // scalar (negate for pedro pathing driving)
        return output;
    }
    public static PIDFCoefficients LimelightAutoAlignmentTurning =
            new PIDFCoefficients(
                    0.03, 0, 0, 0
            );
    public static final double LIMELIGHT_PIDF_MIN_OUTPUT = -1;
    public static final double LIMELIGHT_PIDF_MAX_OUTPUT = 1;
    public static final double LIMELIGHT_PIDF_INTEGRAL_LIMIT = 10;
    public static final double GOAL_HEIGHT_INCHES = 54;
    public static final double LIMELIGHT_LENS_HEIGHT_INCHES = 12.5; // measure
    public static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 12; // measure
    public static final double TRANSFER_UP_POSITION = 0.7;
    public static final double TRANSFER_DOWN_POSITION = 0;
    public static final double VELOCITY_TOLERANCE = 60;
    public static double STOP_VELOCITY = 0; // Set target velocity- in RPM(e.g., 3000 RPM)
    public static PIDFCoefficients LaunchPIDF = new PIDFCoefficients(
            300.0025, 0, 0.0002, 10
    );
    public enum AllianceColor {
        BLUE, RED;

        public AllianceColor switchColors() {
            return this == RED ? BLUE : RED;
        }

        public int getTagID() {
            return this == RED ? 24 : 20;
        }
    }
//    public static Pose RED_GOAL = new Pose(130.37, 127.64, 36);
//    public static Pose BLUE_GOAL = new Pose(13.63, 127.64, 144);
    public static Pose BLUE_GOAL = new Pose(16.3, 130.4, Math.toRadians(144));
    public static Pose RED_GOAL = new Pose(127.7, 130.4, Math.toRadians(36));
    public static Pose RED_SHOOTING_FRONT = new Pose(123.5, 122.8, Math.toRadians(37));
    public static Pose RED_SHOOTING_BACK = new Pose(114, 114, Math.toRadians(45));
    public static Pose BLUE_SHOOTING_FRONT = new Pose(20.5, 122.8, Math.toRadians(143));
    public static Pose BLUE_SHOOTING_BACK = new Pose(30, 114, Math.toRadians(180));
    public static Pose RED_PARKING = new Pose((double) 270/7, (double) 234/7);
    public static Pose BLUE_PARKING = new Pose((double) 738/7, (double) 234/7);
    // PEDRO PATHING CONSTANTS
    public static FollowerConstants pedroFollowerConstants = new FollowerConstants()
            .mass(11.4)
            .forwardZeroPowerAcceleration(-41.031359746341444)
            .lateralZeroPowerAcceleration(-71.62199686407405)
            .translationalPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(0.067, 0, 0.05, 0.01))
            .headingPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(0.9, 0, 0.08, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0,0.0001,3.5,0.00001))
            .centripetalScaling(0.003);
    public static PathConstraints pedroPathConstraints = new PathConstraints(0.99, 100, 1, 1.2);
    public static PinpointConstants pedroLocalizerConstants = new PinpointConstants()
            .forwardPodY(-0.3125)
            .strafePodX(-3.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static final double VELOCITY_SCALING_FACTOR = 1;
    public static MecanumConstants pedroMecanumDriveConstants = new MecanumConstants()
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
            .xVelocity(82.28564164769932*VELOCITY_SCALING_FACTOR)
            .yVelocity(65.39811886764886*VELOCITY_SCALING_FACTOR);

    public static Follower createPedroFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(pedroFollowerConstants, hardwareMap)
                .pathConstraints(pedroPathConstraints)
                .mecanumDrivetrain(pedroMecanumDriveConstants)
                .pinpointLocalizer(pedroLocalizerConstants)
                .build();
    }

}
