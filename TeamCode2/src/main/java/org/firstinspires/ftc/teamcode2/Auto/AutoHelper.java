package org.firstinspires.ftc.teamcode2.Auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import static org.firstinspires.ftc.teamcode2.Systems.Constants.SLEEP_BEFORE_INTAKE_START;
import static org.firstinspires.ftc.teamcode2.Systems.Constants.TARGET_VELOCITY_BACK_LAUNCH_ZONE;
import static org.firstinspires.ftc.teamcode2.Systems.Constants.TARGET_VELOCITY_FRONT_LAUNCH_ZONE;
import static org.firstinspires.ftc.teamcode2.Systems.Constants.TRANSFER_DOWN_POSITION;
import static org.firstinspires.ftc.teamcode2.Systems.Constants.TRANSFER_UP_POSITION;
import static org.firstinspires.ftc.teamcode2.Systems.Constants.VELOCITY_TOLERANCE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode2.Systems.Constants;

import com.pedropathing.util.Timer;

@SuppressWarnings("SpellCheckingInspection")
public class AutoHelper {
    public enum LaunchState {
        IDLE,
        SPINUP_LAUNCHER,
        SPINUP_TRANSFER,
        INTAKE,
        SPIN_DOWN
    }
    public DcMotorEx launcher;
    public DcMotor intake, transfer;
    public Timer launchSequenceTimer;
    public Timer ballLaunchTimer;
    private LaunchState launchState = LaunchState.IDLE;
    private boolean isLaunching;
    int iterations = 0; // Iterations completed
    private void next() {
        launchState = LaunchState.values()[(launchState.ordinal() + 1) % LaunchState.values().length];
        launchSequenceTimer.resetTimer();
    }

    public AutoHelper(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        launcher.setZeroPowerBehavior(FLOAT);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.LaunchPIDF);
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
        double velocity = back? TARGET_VELOCITY_BACK_LAUNCH_ZONE : TARGET_VELOCITY_FRONT_LAUNCH_ZONE;
        if (launchState == LaunchState.IDLE) {
            launcher.setVelocity(velocity - VELOCITY_TOLERANCE);
            launcher.setVelocity(velocity);
            next();
            return false;
        } else if (launchState == LaunchState.SPINUP_LAUNCHER) {
            if (launcher.getVelocity() >= velocity - VELOCITY_TOLERANCE) {
                transfer.setPower(TRANSFER_UP_POSITION);
                next();
                isLaunching = false;
            }
            return false;
        } else if (launchState == LaunchState.SPINUP_TRANSFER) {
            if (launchSequenceTimer.getElapsedTimeSeconds()*1000 >= SLEEP_BEFORE_INTAKE_START) {
                next();
                isLaunching = false;
            }
            return false;
        } else if (launchState == LaunchState.INTAKE) {
            if (iterations>=numBalls) {
                isLaunching = false;
                next();
            } else {
                if (!isLaunching) {
                    // Contains (does the ball need intake)
                    // Intaking the entire time
                    /*
                    if (IntStream.of(Consts.BALL_NUM_INTAKE_NEEDED).anyMatch(num -> num == iterations+1)) {
                        Intake();
                    }
                    */
                    if (ballLaunchTimer.getElapsedTimeSeconds()*1000 >= Constants.SLEEP_BEFORE_INTAKE_RESET_LAUNCHING) {
                        IntakeStop();
                        isLaunching = true;
                        ballLaunchTimer.resetTimer();
                    }
                } else {
                    if (ballLaunchTimer.getElapsedTimeSeconds()*1000 >= Constants.SLEEP_BEFORE_INTAKE_START) {
                        Intake();
                        iterations++;
                        isLaunching = false;
                        ballLaunchTimer.resetTimer();
                    }
                }
            }
            return false;
        } else if (launchState == LaunchState.SPIN_DOWN) {
            launcher.setVelocity(0);
            transfer.setPower(TRANSFER_DOWN_POSITION);
            IntakeStop();
            iterations = 0; // RESET FOR NEXT LAUNCH
            next();
            return true; // Sequence complete
        }
        return false;
    }
}
