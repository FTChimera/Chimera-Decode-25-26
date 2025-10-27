package org.firstinspires.ftc.teamcode.pedroAuto;
import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
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

@Autonomous(name = "BlueAutoPedro", group = "pedroAuto")
public class BlueAutoPedro extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(15.75, 111.27, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose launchPose = new Pose(67, 81, Math.toRadians(315));// Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose intakePrep = new Pose(50,84.7, Math.toRadians(180));
    private final Pose red1Intake = new Pose(15.51, 84.84, Math.toRadians(180));
    private final Pose red2Intake = new Pose(41.69,60.36, Math.toRadians(180));

    private Path pathOne, pathTwo, pathThree, pathFour, pathFive;

    final double TARGET_VELOCITY = 3000; // Set target velocity- in RPM(e.g., 3000 RPM)
    final double TARGET_VELOCITY_BACK_LAUNCH_ZONE = 1085;// Set target velocity from back launch zone
    final double TARGET_VELOCITY_FRONT_LAUNCH_ZONE = 100;// Set target velocity from back launch zone
    final double MIN_VELOCITY_BACK_LAUNCH_ZONE = 10;// Set target velocity from back launch zone
    final double MIN_VELOCITY_FRONT_LAUNCH_ZONE = 50;// Set target velocity from back launch zone
    final double STOP_VELOCITY = 0; // Set target velocity- in RPM(e.g., 3000 RPM)
    final double MIN_VELOCITY = 1075;
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    final int SERVO_LAUNCH_POSITION = -1;
    final int SERVO_REST_POSITION = 1;
    final int SLEEP_BEFORE_RESET_SERVO_POSITION = 500;
    final int CHIMERA_LAUNCH = 1;
    final int CHIMERA_LAUNCH_INTAKE = 2;
    final int CHIMERA_PATH_TWO = 3;
    final int CHIMERA_PATH_THREE = 4;
    final int CHIMERA_PATH_FOUR = 5;
    final int CHIMERA_PATH_FIVE = 6;
    final int CHIMERA_STOP = 7;

    boolean first_iteration = false;




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

        pathTwo = new Path(new BezierLine(launchPose, intakePrep));
        pathTwo.setLinearHeadingInterpolation(launchPose.getHeading(), intakePrep.getHeading());

        pathThree = new Path(new BezierLine(intakePrep, red1Intake));
        pathThree.setLinearHeadingInterpolation(intakePrep.getHeading(), red1Intake.getHeading());

        pathFour = new Path(new BezierLine(red1Intake, launchPose));
        pathFour.setLinearHeadingInterpolation(red1Intake.getHeading(), launchPose.getHeading());

        pathFive = new Path(new BezierLine(launchPose, red2Intake));
        pathFive.setLinearHeadingInterpolation(launchPose.getHeading(), red2Intake.getHeading());



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
                    Launcher();
                    sleep(800);
                    Launcher();
                    setPathState(CHIMERA_LAUNCH_INTAKE);
                }
                break;
            case CHIMERA_LAUNCH_INTAKE:
                Intake();
                sleep(800);
                Launcher();
                if(!first_iteration) {
                    setPathState(CHIMERA_PATH_TWO);
                    first_iteration = true;
                } else {
                    setPathState(CHIMERA_PATH_FIVE);
                }
                break;
            case CHIMERA_PATH_TWO:
                if (!follower.isBusy()) {
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
                }
                break;
            case CHIMERA_PATH_FIVE:
                if (!follower.isBusy()) {
                    follower.followPath(pathFive);
                    IntakeStop();
                    setPathState(CHIMERA_STOP);
                }
                break;
            case CHIMERA_STOP:
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
        opmodeTimer.resetTimer();

        OutakeMotorRight = hardwareMap.get(DcMotorEx.class, "OutakeMotorRight");
        OutakeMotorLeft = hardwareMap.get(DcMotorEx.class, "OutakeMotorLeft");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        pushServo = hardwareMap.servo.get("pushServo");

        OutakeMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        OutakeMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        OutakeMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        OutakeMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        OutakeMotorRight.setZeroPowerBehavior(BRAKE);
        OutakeMotorLeft.setZeroPowerBehavior(BRAKE);

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

    public void Launcher() {

        // Start the timer and motors on the first run
        OutakeMotorRight.setVelocity(TARGET_VELOCITY_BACK_LAUNCH_ZONE);
        OutakeMotorLeft.setVelocity(TARGET_VELOCITY_BACK_LAUNCH_ZONE);
        sleep(200);
        pushServo.setPosition(SERVO_LAUNCH_POSITION);//Set pushServo to launch
        sleep(1200);
        OutakeMotorRight.setVelocity(STOP_VELOCITY);
        OutakeMotorLeft.setVelocity(STOP_VELOCITY);
        pushServo.setPosition(SERVO_REST_POSITION);//Resets the pushServo position
        sleep(1200);

    }
    public void Intake() {
        intakeMotor.setPower(1);
    }
    public void IntakeStop() {
        intakeMotor.setPower(0);
    }
}
