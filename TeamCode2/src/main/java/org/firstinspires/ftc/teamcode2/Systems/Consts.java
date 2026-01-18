package org.firstinspires.ftc.teamcode2.Systems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


public class Consts {
    public static final double DRIVE_SCALAR = 1.0; // Scalar for drive power
    public static final double AUTO_ALIGNMENT_TURN_SCALAR = 0.03; // Scalar for turning in auto-alignment
    public static final double SERVO_UP_POSITION = 0.9;
    public static final double SERVO_DOWN_POSITION = 0;
    public static final double SLEEP_BEFORE_RESET_SERVO_POSITION = 600;
    public static final double SLEEP_BEFORE_SECOND_ITERATION = 400;
    public static final double VELOCITY_TOLERANCE = 100;
    public static double TARGET_VELOCITY_BACK_LAUNCH_ZONE = 1200;// Set target velocity from back launch zone
    public static double TARGET_VELOCITY_FRONT_LAUNCH_ZONE = 1000;// Set target velocity from front launch zone
    public static double MIN_VELOCITY_BACK_LAUNCH_ZONE = 1050;// Set target velocity from back launch zone
    public static double MIN_VELOCITY_FRONT_LAUNCH_ZONE = 890;// Set target velocity from back launch zone
    public static double STOP_VELOCITY = 0; // Set target velocity- in RPM(e.g., 3000 RPM)
    public static int[] BALL_NUM_INTAKE_NEEDED = {3};
    public static PIDFCoefficients LaunchPIDF = new PIDFCoefficients(
            0,0,0,0
    );
    // TODO Change Starting position. Temporarily set starting position to back launch
    // zone, (x,y) = (72,0)
    public enum AllianceColor {
        BLUE, RED;

        public AllianceColor switchColors() {
            return this == RED ? BLUE : RED;
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
