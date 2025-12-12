package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


public class Consts {
    public static double TARGET_VELOCITY_BACK_LAUNCH_ZONE = 1200;// Set target velocity from back launch zone
    public static double TARGET_VELOCITY_FRONT_LAUNCH_ZONE = 900;// Set target velocity from front launch zone
    public static double MIN_VELOCITY_BACK_LAUNCH_ZONE = 400;// Set target velocity from back launch zone
    public static double MIN_VELOCITY_FRONT_LAUNCH_ZONE = 100;// Set target velocity from back launch zone
    public static double STOP_VELOCITY = 0; // Set target velocity- in RPM(e.g., 3000 RPM)
    public static double SERVO_LAUNCH_POSITION = 0.5;
    public static double SERVO_REST_POSITION = 1;
    public static int SLEEP_BEFORE_RESET_SERVO_POSITION = 600;
    public static double maxVelocityLeftOutakeMotor = 1680;
    public static double maxVelocityRightOutakeMotor = 1800;
    public static PIDFCoefficients leftPIDF = getPIDFCoefficients(maxVelocityLeftOutakeMotor,
            1.5, 1, 1, -3);
    public static PIDFCoefficients rightPIDF = getPIDFCoefficients(maxVelocityRightOutakeMotor,
            2, 1, 1, -3);

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
    public static double LAUNCHER_GOALTAG_ANGLE_SCALE = 15; // TUNE
    public static String[] DRIVE_MOTOR_NAMES = {"frontLeftMotor","frontRightMotor","backLeftMotor","backRightMotor"};
    public static PIDFCoefficients getPIDFCoefficients(double maxVelocity, double KpOffset, double KiOffset, double KdOffset, double KfOffset) {
        double Kf = 32767/maxVelocity;
        double Kp = 0.1*Kf;
        double Ki = 0.1*Kp;
        double Kd = 0;
        Kp += KpOffset;
        Ki += KiOffset;
        Kd += KdOffset;
        Kf += KfOffset;
        return new PIDFCoefficients(Kp, Ki, Kd, Kf);
    }
}
