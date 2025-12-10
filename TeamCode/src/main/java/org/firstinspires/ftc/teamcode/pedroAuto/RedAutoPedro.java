package org.firstinspires.ftc.teamcode.pedroAuto;
import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "RedAutoPedro", group = "pedroAuto")
public class RedAutoPedro extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, launcherTimer;
    private int pathState, launcherShotCount = 0;
    private final Pose startPose = new Pose(128.13, 110.81, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose launchPose = new Pose(90.1, 86.3, Math.toRadians(228));// Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose intakePrep = new Pose(100,85, Math.toRadians(0));
    private final Pose red1Intake = new Pose(128, 84, Math.toRadians(0));
    private final Pose intakePrep2 = new Pose(100.38,61.35, Math.toRadians(0));
    private final Pose red2Intake = new Pose(135.02, 55.30, Math.toRadians(0));
    private final Pose intakePrep3 = new Pose(100.27, 37.14, Math.toRadians(0));
    private final Pose red3intake = new Pose (135,36.73, Math.toRadians(0));
    private Path pathOne, pathTwo, pathThree, pathFour, pathFive, pathSix, pathSeven, pathEight, pathNine, pathTen;

    final double TARGET_VELOCITY = 3000; // Set target velocity- in RPM(e.g., 3000 RPM)
    final double TARGET_VELOCITY_BACK_LAUNCH_ZONE = 600;// Set target velocity from back launch zone
    final double TARGET_VELOCITY_FRONT_LAUNCH_ZONE = 100;// Set target velocity from back launch zone
    final double MIN_VELOCITY_BACK_LAUNCH_ZONE = 10;// Set target velocity from back launch zone
    final double MIN_VELOCITY_FRONT_LAUNCH_ZONE = 50;// Set target velocity from back launch zone
    final double STOP_VELOCITY = 0; // Set target velocity- in RPM(e.g., 3000 RPM)
    final double MIN_VELOCITY = 1075;
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    final double SERVO_LAUNCH_POSITION = 0;
    final double SERVO_REST_POSITION = 1;
    final int SLEEP_BEFORE_RESET_SERVO_POSITION = 500;
    final int CHIMERA_LAUNCH = 1;
    final int CHIMERA_LAUNCH_INTAKE = 2;
    final int CHIMERA_PATH_TWO = 3;
    final int CHIMERA_PATH_THREE = 4;
    final int CHIMERA_PATH_FOUR = 5;
    final int CHIMERA_PATH_FIVE = 6;
    final int CHIMERA_PATH_SIX = 7;
    final int CHIMERA_PATH_SEVEN = 8;
    final int CHIMERA_PATH_EIGHT = 9;
    final int CHIMERA_PATH_NINE = 10;
    final int CHIMERA_PATH_TEN = 11;
    final int CHIMERA_STOP = 12;

    boolean first_iteration = false;
    boolean second_iteration = false;
    boolean third_iteration = false;
    boolean isLauncherRunning = false;

    // public static double maxVelocityLeftOutakeMotor = 1680
    // public static double maxVelocityRightOutakeMotor = 1800
    //
    // PID Value
    // kf value = 32767/maxVelocityLeftOutakeMotor
    //      kp value =  0.1 * kf
    //      ki value = 0.1 * kp
    //      kd value = 0

    final double Kp = 3.2767;
    final double Ki = 0.32767;
    final double Kd = 0.032767;
    final double Kf = 32.767;

    DcMotorEx OutakeMotorLeft, OutakeMotorRight;
    DcMotor intakeMotor;
    Servo pushServo;


    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        pathOne = new Path(new BezierLine(startPose, launchPose));
        pathOne.setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading());

        pathTwo = new Path(new BezierLine(launchPose, intakePrep));
        pathTwo.setLinearHeadingInterpolation(launchPose.getHeading(), intakePrep.getHeading());

        pathThree = new Path(new BezierLine(intakePrep, red1Intake));
        pathThree.setLinearHeadingInterpolation(intakePrep.getHeading(), red1Intake.getHeading());

        pathFour = new Path(new BezierLine(red1Intake, launchPose));
        pathFour.setLinearHeadingInterpolation(red1Intake.getHeading(), launchPose.getHeading());

        pathFive = new Path(new BezierLine(launchPose, intakePrep2));
        pathFive.setLinearHeadingInterpolation(launchPose.getHeading(), red2Intake.getHeading());

        pathSix = new Path(new BezierLine(intakePrep2, red2Intake));
        pathSix.setLinearHeadingInterpolation(intakePrep2.getHeading(), red2Intake.getHeading());

        pathSeven = new Path(new BezierLine(red2Intake, launchPose));
        pathSeven.setLinearHeadingInterpolation(red2Intake.getHeading(), launchPose.getHeading());

        pathEight = new Path(new BezierLine(launchPose, intakePrep3));
        pathEight.setLinearHeadingInterpolation(launchPose.getHeading(), intakePrep3.getHeading());

        pathNine = new Path(new BezierLine(intakePrep3, red3intake));
        pathNine.setLinearHeadingInterpolation(intakePrep3.getHeading(), red3intake.getHeading());

        pathTen = new Path(new BezierLine(red3intake, launchPose));
        pathTen.setLinearHeadingInterpolation(red3intake.getHeading(),launchPose.getHeading());


        //add final pos


    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

/// delete if not ready
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(pathOne);
                setPathState(CHIMERA_LAUNCH);
                break;
            case CHIMERA_LAUNCH:
                if (!follower.isBusy()){
                    //Launcher();
                    //sleep(1000);
                    //Launcher();
                    Intake();
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
                    setPathState(CHIMERA_PATH_FIVE);
                    second_iteration = true;
                } else if (!third_iteration) {
                    setPathState(CHIMERA_PATH_EIGHT);
                    third_iteration = true;
                } else {
                    setPathState(CHIMERA_STOP);
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
                if (!follower.isBusy()) {
                    follower.followPath(pathThree);
                    setPathState(CHIMERA_PATH_FOUR);
                }
                break;
            case CHIMERA_PATH_FOUR:
                if (!follower.isBusy()) {
                    follower.followPath(pathFour);
                    setPathState(CHIMERA_LAUNCH);
                    IntakeStop();
                }
                break;
            case CHIMERA_PATH_FIVE:
                if (!follower.isBusy()) {
                    Intake();
                    follower.followPath(pathFive);
                    setPathState(CHIMERA_PATH_SIX);
                }
                break;
            case CHIMERA_PATH_SIX:
                if (!follower.isBusy()) {
                    follower.followPath(pathSix);
                    setPathState(CHIMERA_PATH_SEVEN);
                }
                break;
            case CHIMERA_PATH_SEVEN:
                if (!follower.isBusy()) {
                    follower.followPath(pathSeven);
                    setPathState(CHIMERA_LAUNCH);
                }
                break;
            case CHIMERA_PATH_EIGHT:
                if (!follower.isBusy()) {
                    Intake();
                    follower.followPath(pathEight);
                    setPathState(CHIMERA_PATH_NINE);
                }
                break;
            case CHIMERA_PATH_NINE:
                if(!follower.isBusy()) {
                    follower.followPath(pathNine);
                    setPathState(CHIMERA_PATH_TEN);
                }
                break;
            case CHIMERA_PATH_TEN:
                if(!follower.isBusy()){
                    follower.followPath(pathTen);
                    setPathState(CHIMERA_LAUNCH);
                }
                break;
            case CHIMERA_STOP:
                telemetry.addLine("Autonomous Complete");
                IntakeStop();
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

        OutakeMotorRight.setZeroPowerBehavior(FLOAT);
        OutakeMotorLeft.setZeroPowerBehavior(FLOAT);

        OutakeMotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Kp, Ki, Kd, Kf));
        OutakeMotorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Kp, Ki, Kd, Kf));

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

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}


    /**
     * Executes 3 shots.
     * - Includes Timeout Fail-Safe for low battery.
     * - Turns on Intake automatically to feed shots 2 and 3.
     */
    public boolean runLauncherSequence() {
        // 1. Initialization (Start the flywheels)
        if (!isLauncherRunning) {
            OutakeMotorRight.setVelocity(TARGET_VELOCITY_BACK_LAUNCH_ZONE);
            OutakeMotorLeft.setVelocity(TARGET_VELOCITY_BACK_LAUNCH_ZONE);

            // Only start shooting if motors are up to speed
            if (OutakeMotorRight.getVelocity() >= TARGET_VELOCITY_BACK_LAUNCH_ZONE &&
                    OutakeMotorLeft.getVelocity() >= TARGET_VELOCITY_BACK_LAUNCH_ZONE) {

                isLauncherRunning = true;
                launcherShotCount = 0;
                launcherTimer.resetTimer();
            } else {
                // Wait for motors to spin up
                return false;
            }
        }

        // 2. Logic for shots based on time
        // We want 3 shots. Each shot cycle takes about 1000ms (500ms extend, 500ms retract)
        double time = launcherTimer.getElapsedTimeSeconds(); // Get seconds
        double cycleTime = (SLEEP_BEFORE_RESET_SERVO_POSITION * 2) / 1000.0; // e.g., 1.0 second

        if (launcherShotCount < 3) {
            // Determine where we are in the current shot cycle
            double timeInCurrentCycle = time - (launcherShotCount * cycleTime);

            if (timeInCurrentCycle < (SLEEP_BEFORE_RESET_SERVO_POSITION / 1000.0)) {
                // First half of cycle: PUSH
                pushServo.setPosition(SERVO_LAUNCH_POSITION);
            } else {
                // Second half of cycle: REST
                pushServo.setPosition(SERVO_REST_POSITION);
            }

            // Check if we finished a full cycle
            if (timeInCurrentCycle >= cycleTime) {
                launcherShotCount++;
            }
            return false; // Not done yet
        } else {
            // 3. Cleanup: We have finished 3 shots
            OutakeMotorRight.setVelocity(STOP_VELOCITY);
            OutakeMotorLeft.setVelocity(STOP_VELOCITY);
            pushServo.setPosition(SERVO_REST_POSITION);
            isLauncherRunning = false; // Reset flag for next time we call this
            return true; // We are done!
        }
    }


    /* public void Launcher() {
        // Start the timer and motors on the first run
        OutakeMotorRight.setVelocity(TARGET_VELOCITY_BACK_LAUNCH_ZONE);
        OutakeMotorLeft.setVelocity(TARGET_VELOCITY_BACK_LAUNCH_ZONE);
        while (OutakeMotorRight.getVelocity() < TARGET_VELOCITY_BACK_LAUNCH_ZONE)
            continue;
        while (OutakeMotorLeft.getVelocity() < TARGET_VELOCITY_BACK_LAUNCH_ZONE)
            continue;
        pushServo.setPosition(SERVO_LAUNCH_POSITION);//Set pushServo to launch
        while (pushServo.getPosition() > SERVO_LAUNCH_POSITION)
            continue;
        sleep(SLEEP_BEFORE_RESET_SERVO_POSITION);
        pushServo.setPosition(SERVO_REST_POSITION);//Resets the pushServo position
        while (pushServo.getPosition() < SERVO_REST_POSITION)
            continue;
        sleep(SLEEP_BEFORE_RESET_SERVO_POSITION);

        pushServo.setPosition(SERVO_LAUNCH_POSITION);//Set pushServo to launch
        while (pushServo.getPosition() > SERVO_LAUNCH_POSITION)
            continue;
        sleep(SLEEP_BEFORE_RESET_SERVO_POSITION);
        pushServo.setPosition(SERVO_REST_POSITION);//Resets the pushServo position
        while (pushServo.getPosition() < SERVO_REST_POSITION)
            continue;
        sleep(1000);

        pushServo.setPosition(SERVO_LAUNCH_POSITION);//Set pushServo to launch
        while (pushServo.getPosition() > SERVO_LAUNCH_POSITION)
            continue;
        sleep(SLEEP_BEFORE_RESET_SERVO_POSITION);
        pushServo.setPosition(SERVO_REST_POSITION);//Resets the pushServo position
        while (pushServo.getPosition() < SERVO_REST_POSITION)
            continue;

    }*/

    public void LauncherStop() {
        OutakeMotorRight.setVelocity(STOP_VELOCITY);
        OutakeMotorLeft.setVelocity(STOP_VELOCITY);
        pushServo.setPosition(SERVO_REST_POSITION);//Resets the pushServo position
        //sleep(400);
    }
    public void Intake() {
        intakeMotor.setPower(1);
    }
    public void IntakeStop() {
        intakeMotor.setPower(0);
    }
}