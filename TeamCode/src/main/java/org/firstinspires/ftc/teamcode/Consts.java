package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class Consts {
    public final double TARGET_VELOCITY = 3000; // Set target velocity- in RPM(e.g., 3000 RPM)
    public final double TARGET_VELOCITY_BACK_LAUNCH_ZONE = 1170;// Set target velocity from back launch zone
    public final double TARGET_VELOCITY_FRONT_LAUNCH_ZONE = 800;// Set target velocity from front launch zone
    public final double MIN_VELOCITY_BACK_LAUNCH_ZONE = 900;// Set target velocity from back launch zone
    public final double MIN_VELOCITY_FRONT_LAUNCH_ZONE = 500;// Set target velocity from back launch zone
    public final double STOP_VELOCITY = 0; // Set target velocity- in RPM(e.g., 3000 RPM)
    public final double MIN_VELOCITY = 1075;
    public final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    public final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    public final double FULL_SPEED = 1.0;
    public final int SERVO_LAUNCH_POSITION = 0;
    public final int SERVO_REST_POSITION = 1;
    public final int SLEEP_BEFORE_RESET_SERVO_POSITION = 600;

    // declaring our PIDF tuning values
    public final double Kp = 300;
    public final double Ki = 0.0;
    public final double Kd = 0.0;
    public final double Kf = 10;
    public double setTargetVelocity = 0;
    public double setMinVelocity = 0;
    public static Pose startingPose;
    public double X_Coordinate_Blue_Goal = 0;
    public double Y_Coordinate_Blue_Goal = 144;
    public double X_Coordinate_Red_Goal = 144;
    public double Y_Coordinate_Red_Goal = 144;
    public final double SHOOTING_ZONE_CLOSE_FRONT_LAUNCH_ZONE = 20;
    public final double SHOOTING_ZONE_FAR_FRONT_LAUNCH_ZONE = 60;
    public final double SHOOTING_ZONE_BACK_LAUNCH_ZONE = 65;

    // TODO Change Starting position. Temporarily set starting position to back launch
    // zone, (x,y) = (72,0)
    public final double RED_ALLIANCE_STARTING_X_COORDINATE = 104;
    public final double RED_ALLIANCE_STARTING_Y_COORDINATE = 60;
    public final double RED_ALLIANCE_STARTING_HEADING_POSITION = 180;

    public final double BLUE_ALLIANCE_STARTING_X_COORDINATE = 144;
    public final double BLUE_ALLIANCE_STARTING_Y_COORDINATE = 0;
    public final double BLUE_ALLIANCE_STARTING_HEADING_POSITION = 90;
    public enum AllianceColor {BLUE, RED} AllianceColor allianceColor;
}
