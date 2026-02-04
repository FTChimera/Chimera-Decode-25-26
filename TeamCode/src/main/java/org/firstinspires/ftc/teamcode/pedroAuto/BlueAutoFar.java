package org.firstinspires.ftc.teamcode.pedroAuto;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Systems.Consts;

@Autonomous(name = "BlueAutoFar", group = "pedroAuto")
public class BlueAutoFar extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, launcherTimer;
    private int pathState, launcherShotCount = 0, launcherStage = 0;
    private final Pose startPose = new Pose(56.2, 8.2, Math.toRadians(270)); // Start Pose of our robot.
    private final Pose launchPose = new Pose(56.7, 17.2, Math.toRadians(294));// Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose blue1Intake = new Pose(10.2,9.1,Math.toRadians(180));
    private final Pose launchControl = new Pose(56,18.9,Math.toRadians(180));
    private final Pose finalpose = new Pose(37.4, 8.8, Math.toRadians(90));


    private Path pathOne, pathTwo, pathThree, pathFour, pathFive, pathSix, pathSeven, pathEight, pathNine, pathTen;

    final double TARGET_VELOCITY_BACK_LAUNCH_ZONE = 1250;// Set target velocity from back launch zone
    final double TARGET_VELOCITY_TOLERANCE = 15;
    final double STOP_VELOCITY = 0; // Set target velocity- in RPM(e.g., 3000 RPM)
    final double SERVO_LAUNCH_POSITION = 0.5;
    final int SERVO_REST_POSITION = 1;
    final int SLEEP_BEFORE_RESET_SERVO_POSITION = 1500;
    final int MAX_RPM_WAIT_TIME_SECONDS = 1000;
    final int CHIMERA_LAUNCH = 1;
    final int CHIMERA_LAUNCH_INTAKE = 2;
    final int CHIMERA_PATH_TWO = 3;
    final int CHIMERA_PATH_THREE = 4;
    final int CHIMERA_PATH_FOUR = 5;
    final int CHIMERA_PATH_FIVE = 6;
    final int CHIMERA_PATH_SIX = 7;
    final int CHIMERA_PATH_SEVEN = 8;
    final int CHIMERA_PATH_EIGHT = 9;
    final int CHIMERA_STOP = 10;

    boolean first_iteration = false;
    boolean second_iteration = false;
    boolean third_iteration = false;
    boolean isLauncherRunning = false;

    final double Kp = 300;
    final double Ki = 0.0;
    final double Kd = 0.0;
    final double Kf = 10;

    DcMotorEx OutakeMotorLeft, OutakeMotorRight;
    DcMotor intakeMotor;
    Servo pushServo;


    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        pathOne = new Path(new BezierLine(startPose, launchPose));
        pathOne.setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading());

        pathTwo = new Path(new BezierCurve(launchPose, launchControl, blue1Intake));
        pathTwo.setLinearHeadingInterpolation(launchPose.getHeading(), blue1Intake.getHeading());

        pathThree = new Path(new BezierLine(blue1Intake, launchPose));
        pathThree.setLinearHeadingInterpolation(blue1Intake.getHeading(), launchPose.getHeading());

        pathFour = new Path(new BezierLine(launchPose, finalpose));
        pathFour.setLinearHeadingInterpolation(launchPose.getHeading(), finalpose.getHeading());

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(pathOne);
                setPathState(CHIMERA_LAUNCH);
                break;
            case CHIMERA_LAUNCH:
                if (!follower.isBusy()){
                    if (runLauncherSequence()) {
                        setPathState(CHIMERA_LAUNCH_INTAKE);
                    }
                }
                break;
            case CHIMERA_LAUNCH_INTAKE:
                IntakeStop();
                LauncherStop();
                if(!first_iteration) {
                    setPathState(CHIMERA_PATH_TWO);
                    first_iteration = true;
                } else if (!second_iteration) {
                   setPathState(CHIMERA_PATH_FOUR);
                   second_iteration = true;
                }

                break;
            case CHIMERA_PATH_TWO:
                if (!follower.isBusy()) {
                    Intake();
                    follower.followPath(pathTwo);
                    setPathState(CHIMERA_PATH_THREE);
                }
                break;
            case CHIMERA_PATH_THREE:
                if (!follower.isBusy()){
                    IntakeStop();
                    follower.followPath(pathThree);
                    setPathState(CHIMERA_LAUNCH);
                }
                break;
            case CHIMERA_PATH_FOUR:
                if(!follower.isBusy()) {
                    follower.followPath(pathFour);
                    setPathState(CHIMERA_STOP);
                }
            case CHIMERA_STOP:
                IntakeStop();
                LauncherStop();
                telemetry.addLine("Autonomous Complete");
            default:
                break;
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
        launcherTimer = new Timer();

        opmodeTimer.resetTimer();

        OutakeMotorRight = hardwareMap.get(DcMotorEx.class, "OutakeMotorRight");
        OutakeMotorLeft = hardwareMap.get(DcMotorEx.class, "OutakeMotorLeft");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        pushServo = hardwareMap.servo.get("pushServo");

        OutakeMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        OutakeMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        OutakeMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        OutakeMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        OutakeMotorRight.setZeroPowerBehavior(BRAKE);
        OutakeMotorLeft.setZeroPowerBehavior(BRAKE);

        OutakeMotorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Consts.leftPIDF);
        OutakeMotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Consts.rightPIDF);

        pushServo.setPosition(SERVO_REST_POSITION);

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

    @Override
    public void stop() {}

    public boolean runLauncherSequence() {
        // 1. Initialization (Start the flywheels)
        if (!isLauncherRunning) {
            OutakeMotorRight.setVelocity(TARGET_VELOCITY_BACK_LAUNCH_ZONE);
            OutakeMotorLeft.setVelocity(TARGET_VELOCITY_BACK_LAUNCH_ZONE);

            isLauncherRunning = true;
            launcherShotCount = 0;
            launcherStage = 0;
            launcherTimer.resetTimer();
            return false;
        }

        // 2. Sequence Completion (Safety Stop)
        if (launcherShotCount >= 3) {
            OutakeMotorRight.setVelocity(STOP_VELOCITY);
            OutakeMotorLeft.setVelocity(STOP_VELOCITY);
            pushServo.setPosition(SERVO_REST_POSITION);

            // IMPORTANT: Turn off the intake when we are done!
            IntakeStop();

            isLauncherRunning = false;
            return true; // Return TRUE to tell the main loop we are finished
        }

        // 3. The Shot Logic State Machine
        switch (launcherStage) {
            case 0: // STAGE: RECOVERING RPM & FEEDING
                pushServo.setPosition(SERVO_REST_POSITION);

                // --- INTAKE LOGIC ---
                // If we are on Shot 2 (index 1) or Shot 3 (index 2), we need to feed the ring.
                // We do this while waiting for the motors to spin up.
                if (launcherShotCount > 0) {
                    Intake();
                } else {
                    // Keep intake off for the very first shot (assuming it's pre-loaded)
                    IntakeStop();
                }
                // --------------------

                double currentVelR = OutakeMotorRight.getVelocity();
                double currentVelL = OutakeMotorLeft.getVelocity();
                double targetThreshold = TARGET_VELOCITY_BACK_LAUNCH_ZONE - TARGET_VELOCITY_TOLERANCE;

                // Check time for Fail-Safe
                double timeWaiting = launcherTimer.getElapsedTimeSeconds();

                boolean isSpeedReached = (currentVelR >= targetThreshold && currentVelL >= targetThreshold);
                boolean isTimedOut = (timeWaiting > MAX_RPM_WAIT_TIME_SECONDS);

                // If speed is good OR we waited too long (fail-safe)
                if (isSpeedReached || isTimedOut) {

                    // Optional: Stop intake right before shooting to prevent jamming?
                    // Depending on your robot, you might want to comment this line out
                    // if you want the intake to keep pushing during the shot.
                    // IntakeStop();

                    launcherStage = 1;
                    launcherTimer.resetTimer();
                }
                break;

            case 1: // STAGE: PUSHING
                pushServo.setPosition(SERVO_LAUNCH_POSITION);

                // Wait for the servo to physically reach the position
                if (launcherTimer.getElapsedTimeSeconds() >= (SLEEP_BEFORE_RESET_SERVO_POSITION / 1000.0)) {
                    launcherStage = 2;
                    launcherTimer.resetTimer();
                }
                break;

            case 2: // STAGE: RESETTING
                pushServo.setPosition(SERVO_REST_POSITION);

                // Wait for servo to pull back
                if (launcherTimer.getElapsedTimeSeconds() >= (SLEEP_BEFORE_RESET_SERVO_POSITION / 1000.0)) {
                    launcherShotCount++; // Increment shot count
                    launcherStage = 0;   // Loop back to Stage 0
                }
                break;
        }

        return false; // Not done yet
    }
    public void LauncherStop() {
        OutakeMotorRight.setVelocity(STOP_VELOCITY);
        OutakeMotorLeft.setVelocity(STOP_VELOCITY);
        pushServo.setPosition(SERVO_REST_POSITION);//Resets the pushServo position
        //
        // sleep(200);
    }
    public void Intake() {
        intakeMotor.setPower(1);
    }
    public void IntakeStop() {
        intakeMotor.setPower(0);
    }
}