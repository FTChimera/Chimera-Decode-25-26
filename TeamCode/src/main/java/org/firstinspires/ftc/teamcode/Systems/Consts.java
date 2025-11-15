package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.geometry.Pose;

public class Consts {
    public static double TARGET_VELOCITY = 3000; // Set target velocity- in RPM(e.g., 3000 RPM)
    public static double TARGET_VELOCITY_BACK_LAUNCH_ZONE = 1170;// Set target velocity from back launch zone
    public static double TARGET_VELOCITY_FRONT_LAUNCH_ZONE = 800;// Set target velocity from front launch zone
    public static double MIN_VELOCITY_BACK_LAUNCH_ZONE = 900;// Set target velocity from back launch zone
    public static double MIN_VELOCITY_FRONT_LAUNCH_ZONE = 500;// Set target velocity from back launch zone
    public static double STOP_VELOCITY = 0; // Set target velocity- in RPM(e.g., 3000 RPM)
    public static double MIN_VELOCITY = 1075;
    public static double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    public static double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    public static double FULL_SPEED = 1.0;
    public static int SERVO_LAUNCH_POSITION = 0;
    public static int SERVO_REST_POSITION = 1;
    public static int SLEEP_BEFORE_RESET_SERVO_POSITION = 600;

    // declaring our PIDF tuning values
    public static double Kp = 300;
    public static double Ki = 0.0;
    public static double Kd = 0.0;
    public static double Kf = 10;
    public double setTargetVelocity = 0;
    public double setMinVelocity = 0;
    public static Pose startingPose;
    public static double X_Coordinate_Blue_Goal = 0;
    public static double Y_Coordinate_Blue_Goal = 144;
    public static double X_Coordinate_Red_Goal = 144;
    public static double Y_Coordinate_Red_Goal = 144;
    public static double LL_GOALTAG_X_COORDINATE = 0;
    public static double LL_GOALTAG_Y_COORDINATE = 20; // TODO: TUNE
    public static double SHOOTING_ZONE_CLOSE_FRONT_LAUNCH_ZONE = 20;
    public static double SHOOTING_ZONE_FAR_FRONT_LAUNCH_ZONE = 60;
    public static double SHOOTING_ZONE_BACK_LAUNCH_ZONE = 65;

    // TODO Change Starting position. Temporarily set starting position to back launch
    // zone, (x,y) = (72,0)
    public static final double RED_ALLIANCE_STARTING_X_COORDINATE = 104;
    public static final double RED_ALLIANCE_STARTING_Y_COORDINATE = 60;
    public static final double RED_ALLIANCE_STARTING_HEADING_POSITION = 180;

    public static final double BLUE_ALLIANCE_STARTING_X_COORDINATE = 144;
    public static final double BLUE_ALLIANCE_STARTING_Y_COORDINATE = 0;
    public static final double BLUE_ALLIANCE_STARTING_HEADING_POSITION = 90;
    public enum AllianceColor {BLUE, RED}
    public static Pose RED_STARTING_POSE = new Pose(RED_ALLIANCE_STARTING_X_COORDINATE, RED_ALLIANCE_STARTING_Y_COORDINATE, Math.toRadians(RED_ALLIANCE_STARTING_HEADING_POSITION));
    public static Pose BLUE_STARTING_POSE = new Pose(BLUE_ALLIANCE_STARTING_X_COORDINATE, BLUE_ALLIANCE_STARTING_Y_COORDINATE, Math.toRadians(BLUE_ALLIANCE_STARTING_HEADING_POSITION));
    public static double LAUNCHER_GOALTAG_OFFSET = 30; // TUNE
    public static double LAUNCHER_GOALTAG_ANGLE_SCALE = 30;
}
