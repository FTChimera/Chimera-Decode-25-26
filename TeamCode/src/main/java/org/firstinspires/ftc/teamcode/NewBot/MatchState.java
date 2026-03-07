package org.firstinspires.ftc.teamcode.NewBot;

import com.pedropathing.geometry.Pose;

public class MatchState {
    // keep static values to keep values in memory
    // for transfer from auto to teleop

    public static Pose PEDRO_END_POSE = new Pose();
    public static boolean wasFarAuto;
    public static Constants.AllianceColor allianceColor;

    public static void setEndPose(Pose endPose) {
        PEDRO_END_POSE = endPose;
    }

    public static void setAutoTypeInfo(Constants.AllianceColor alliance, boolean farAuto) {
        wasFarAuto = farAuto; allianceColor = alliance;
    }

    public static Pose getStartingPose() {
        return PEDRO_END_POSE;
    }

    public static Constants.AllianceColor getAllianceColor() {
        return allianceColor;
    }
}
