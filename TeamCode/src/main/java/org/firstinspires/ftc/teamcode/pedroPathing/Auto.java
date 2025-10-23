package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Autonomous(name = " Auto", group = "pedroPathing")
public class Auto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(130, 113, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose firstArtifact = new Pose(75, 81, Math.toRadians(50)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private Path pathOne;


    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        pathOne = new Path(new BezierLine(startPose, firstArtifact));
        pathOne.setLinearHeadingInterpolation(startPose.getHeading(), firstArtifact.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

/// delete if not ready
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(pathOne);
                setPathState(1);


                break;
            case 1:
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}


}