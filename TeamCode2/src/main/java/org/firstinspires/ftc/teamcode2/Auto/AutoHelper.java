package org.firstinspires.ftc.teamcode2.Auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode2.Systems.Consts;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.stream.IntStream;

public class AutoHelper {
    public enum LaunchState {
        IDLE,
        SPINUP,
        LAUNCHING,
        SPIN_DOWN
    }
    public DcMotorEx launcher;
    public DcMotor intake;
    CRServo pushServo;
    public Timer launchSequenceTimer;
    public Timer ballLaunchTimer;
    private LaunchState launchState = LaunchState.IDLE;
    private boolean isServoLaunching;
    int iterations = 0; // Iterations completed
    private void next() {
        launchState = LaunchState.values()[(launchState.ordinal() + 1) % LaunchState.values().length];
        launchSequenceTimer.resetTimer();
    }

    public AutoHelper(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        pushServo = hardwareMap.get(CRServo.class, "pushServo");

        launcher.setZeroPowerBehavior(FLOAT);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Consts.LaunchPIDF);
        launchSequenceTimer = new Timer();
        ballLaunchTimer = new Timer();
    }
    public void Intake() {
        intake.setPower(1.0);
    }

    public void IntakeStop() {
        intake.setPower(0.0);
    }
    public void IntakeReverse() {
        intake.setPower(-1.0);
    }

    public void Intake(double power) {
        intake.setPower(power);
    }

    // The boolean is to indicate whether the sequence was completed (false: continue, true: sequence done)
    public boolean runLauncherSequence(boolean back, int numBalls) {
        if (launchState == LaunchState.IDLE) {
            // Find velocity based on back
            if (back) {
                launcher.setVelocity(Consts.MIN_VELOCITY_BACK_LAUNCH_ZONE);
                launcher.setVelocity(Consts.TARGET_VELOCITY_BACK_LAUNCH_ZONE);
            } else {
                launcher.setVelocity(Consts.MIN_VELOCITY_FRONT_LAUNCH_ZONE);
                launcher.setVelocity(Consts.TARGET_VELOCITY_FRONT_LAUNCH_ZONE);
            }
            next();
            return false;
        } else if (launchState == LaunchState.SPINUP) {
            if (launcher.getVelocity() >= Consts.TARGET_VELOCITY_FRONT_LAUNCH_ZONE - Consts.VELOCITY_TOLERANCE) {
                next();
                isServoLaunching = false;
            }
            return false;
        } else if (launchState == LaunchState.LAUNCHING) {
            if (iterations>=numBalls) {
                isServoLaunching = false;
                next();
            } else {
                if (!isServoLaunching) {
                    // Contains (does the ball need intake)
                    if (IntStream.of(Consts.BALL_NUM_INTAKE_NEEDED).anyMatch(num -> num == iterations+1)) {
                        Intake();
                    }
                    if (ballLaunchTimer.getElapsedTimeSeconds()/1000 >= Consts.SLEEP_BEFORE_SECOND_ITERATION) {
                        IntakeStop();
                        pushServo.setPower(Consts.SERVO_UP_POSITION);
                        isServoLaunching = true;
                        ballLaunchTimer.resetTimer();
                    }
                } else {
                    if (ballLaunchTimer.getElapsedTimeSeconds()/1000 >= Consts.SLEEP_BEFORE_RESET_SERVO_POSITION) {
                        pushServo.setPower(Consts.SERVO_DOWN_POSITION);
                        iterations++;
                        isServoLaunching = false;
                        ballLaunchTimer.resetTimer();
                    }
                }
            }
            return false;
        } else if (launchState == LaunchState.SPIN_DOWN) {
            launcher.setVelocity(0);
            iterations = 0; // RESET FOR NEXT LAUNCH
            next();
            return true; // Sequence complete
        }
        return false;
    }
}
