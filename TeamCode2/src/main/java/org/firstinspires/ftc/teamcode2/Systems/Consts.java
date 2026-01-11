package org.firstinspires.ftc.teamcode2.Systems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


public class Consts {
    public static double TARGET_VELOCITY_BACK_LAUNCH_ZONE = 1200;// Set target velocity from back launch zone
    public static double TARGET_VELOCITY_FRONT_LAUNCH_ZONE = 1000;// Set target velocity from front launch zone
    public static double MIN_VELOCITY_BACK_LAUNCH_ZONE = 400;// Set target velocity from back launch zone
    public static double MIN_VELOCITY_FRONT_LAUNCH_ZONE = 100;// Set target velocity from back launch zone
    public static double STOP_VELOCITY = 0; // Set target velocity- in RPM(e.g., 3000 RPM)
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
    public static Pose TELEOP_RED_STARTING_POSE = new Pose(84, 36, Math.toRadians(50));
    public static Pose TELEOP_BLUE_STARTING_POSE = new Pose(35.1, 79.7, Math.toRadians(312));
    public static Pose AUTO_RED_STARTING_POSE = new Pose(128.13, 110.81, Math.toRadians(0));
    public static Pose AUTO_BLUE_STARTING_POSE = new Pose(15.75, 111.27, Math.toRadians(180));
    public static Pose RED_GOAL = new Pose(130.37, 127.64, Math.toRadians(45));
    public static Pose BLUE_GOAL = new Pose(13.63, 127.64, Math.toRadians(135));
}
