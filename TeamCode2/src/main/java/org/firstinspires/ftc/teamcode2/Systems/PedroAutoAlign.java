package org.firstinspires.ftc.teamcode2.Systems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

public class PedroAutoAlign {
    // Test class of how this works
    // requires pedro teleop drive
    public Pose currentPose, newPose;
    double Posex, Posey, diffX, diffY;
    double angle;
    public Consts.AllianceColor currentGoal;

    public Follower follower;
    Path autoAlignmentPath;
    Pose GoalCoordinates;

    public PedroAutoAlign(Follower follower1, Consts.AllianceColor allianceColor) {
        follower = follower1;
        currentGoal = allianceColor;
        if (currentGoal == Consts.AllianceColor.RED) {
            GoalCoordinates = RedGoalCoordinates;
        } else {
            GoalCoordinates = BlueGoalCoordinates;
        }
    }

    public Pose RedGoalCoordinates = new Pose(
            0,144
    );
    public Pose BlueGoalCoordinates = new Pose(
            144, 144
    );

    // main function to auto align to goal
    public void turnAutoAlign() {
        currentPose = follower.getPose();
        Posex = currentPose.getX();
        Posey = currentPose.getY();

        // Find angle using tan inverse
        // find side lengths

        diffX = Math.abs(Posex - GoalCoordinates.getX());
        diffY = Math.abs(Posey - GoalCoordinates.getY());

        // tan theta = opposite divided by adjacent = y/x
        // same as atan2(double, double)

        angle = Math.atan2(diffY, diffX);

        // set path
        newPose = new Pose(
                Posex, Posey, angle
        );
        autoAlignmentPath = new Path(
                new BezierLine(currentPose, newPose)
        );
        autoAlignmentPath.setLinearHeadingInterpolation(
                currentPose.getHeading(), angle
        );

        // follow path
        follower.followPath(autoAlignmentPath);
    }
}
