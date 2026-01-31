package org.firstinspires.ftc.teamcode2.Systems;

import static org.firstinspires.ftc.teamcode2.Systems.Consts.AllianceColor.BLUE;
import static org.firstinspires.ftc.teamcode2.Systems.Consts.AllianceColor.RED;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@SuppressWarnings("SpellCheckingInspection")
public class Consts {

    public static double applyPolynomialToDriveInputs(double input) {
        double output = input;
        output = output*output; // square for more control
        output = output*-1.0; // scalar (negate for pedro pathing)
        return output;
    }
    public static final double AUTO_ALIGNMENT_TURN_SCALAR = 0.03; // Scalar for turning in auto-alignment
    // TUNE LL_PIDF VALUES FOR AUTO-ALIGNMENT
    public static PIDFCoefficients LimelightAutoAlignmentTurning =
        new PIDFCoefficients(
                0.03, 0.0, 0.0, 0.5
        );
    public static final double LIMELIGHT_PIDF_MIN_OUTPUT = -1;
    public static final double LIMELIGHT_PIDF_MAX_OUTPUT = 1;
    public static final double LIMELIGHT_PIDF_INTEGRAL_LIMIT = 10;


    public static final double SERVO_UP_POSITION = 0.9;
    public static final double SERVO_DOWN_POSITION = 0; // Use CR Servo
    public static final double SLEEP_BEFORE_RESET_SERVO_POSITION = 600;
    public static final double SLEEP_BEFORE_SECOND_ITERATION = 400;
    public static final double VELOCITY_TOLERANCE = 70;
    // 550 test for now. larger amounts will break the wood for now
    public static double TARGET_VELOCITY_BACK_LAUNCH_ZONE = 550;// Set target velocity from back launch zone
    public static double TARGET_VELOCITY_FRONT_LAUNCH_ZONE = 550;// Set target velocity from front launch zone
    public static double STOP_VELOCITY = 0; // Set target velocity- in RPM(e.g., 3000 RPM)
    public static int[] BALL_NUM_INTAKE_NEEDED = {2,3};
    public static PIDFCoefficients LaunchPIDF = new PIDFCoefficients(
            10,0,0,0
    );
    public enum AllianceColor {
        BLUE, RED;

        public AllianceColor switchColors() {
            return this == RED ? BLUE : RED;
        }
    }
    public enum Auto {
        BLUE_CLOSE(BLUE),
        BLUE_FAR(BLUE),
        RED_CLOSE(RED),
        RED_FAR(RED);

        Auto(AllianceColor allianceColor) {

        }
        public Auto next() {
            return values()[(this.ordinal() + 1) % values().length];
        }
        public AllianceColor getAllianceColor() {
            return name().startsWith("BLUE")
                    ? BLUE
                    : RED;
        }
    }
    public static Pose RED_GOAL = new Pose(130.37, 127.64, Math.toRadians(45));
    public static Pose BLUE_GOAL = new Pose(13.63, 127.64, Math.toRadians(135));
    public static Pose RED_SHOOTING_FRONT = new Pose(85, 40, Math.toRadians(0));
    public static Pose RED_SHOOTING_BACK = new Pose(45, 98, Math.toRadians(0));
    public static Pose BLUE_SHOOTING_FRONT = new Pose(60, 85, Math.toRadians(180));
    public static Pose BLUE_SHOOTING_BACK = new Pose(100, 40, Math.toRadians(180));
    public static Pose RED_PARKING = new Pose((double) 270/7, (double) 234/7);
    public static Pose BLUE_PARKING = new Pose((double) 738/7, (double) 234/7);
}
