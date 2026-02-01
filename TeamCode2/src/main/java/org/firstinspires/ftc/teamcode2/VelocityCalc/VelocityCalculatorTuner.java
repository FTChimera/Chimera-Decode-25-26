package org.firstinspires.ftc.teamcode2.VelocityCalc;

import android.annotation.SuppressLint;
import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode2.Auto.RED_AUTO_ARCHIVE;
import org.firstinspires.ftc.teamcode2.Systems.Consts;
import org.firstinspires.ftc.teamcode2.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode2.Systems.RGBIndicator;
import org.firstinspires.ftc.teamcode2.Systems.TeleOpDriveControl;
import org.firstinspires.ftc.teamcode2.pedroPathing.Constants;

import com.pedropathing.util.Timer;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("SpellCheckingInspection")
@TeleOp(name="Velocity Calc Tuner", group="Tests")
public class VelocityCalculatorTuner extends OpMode {
    // Drive and hardware
    TeleOpDriveControl drive;
    DcMotor intake; 
    DcMotorEx launcher; 
    CRServo pushServo;
    boolean oneGamepadControl = true;
    Timer pushServo_timer;
    LimelightSystem limelight;
    RGBIndicator rgbIndicator;
    Follower follower;
    VelocityCalculator velocityCalc;
    
    // Tuning data collection
    private List<TuningDataPoint> tuningData = new ArrayList<>();
    private double targetVelocity;
    private boolean isRecording = false;
    private double currentDistance = 0;
    
    // Servo state machine
    private enum ServoState { IDLE, GOING_UP, LAUNCHING, GOING_DOWN }
    private ServoState servoState = ServoState.IDLE;
    
    // Data point for tuning
    private static class TuningDataPoint {
        public double distance;
        public double velocity;
        public boolean successful;
        
        public TuningDataPoint(double dist, double vel, boolean success) {
            this.distance = dist;
            this.velocity = vel;
            this.successful = success;
        }
        
        @SuppressLint("DefaultLocale")
        @NonNull
        @Override
        public String toString() {
            return String.format("%.1f to %.0f%s", distance, velocity, successful ? " ✓" : " ✗");
        }
    }

    public void resetTimer(Timer timer) {
        if (timer != null) {
            timer.resetTimer();
        } else {
            timer = new Timer();
            timer.resetTimer();
        }
    }

    public void setVelocity(double vel) {
        launcher.setVelocity(vel);
        targetVelocity = vel;
    }

    @Override
    public void init() {
        limelight = new LimelightSystem(hardwareMap);
        rgbIndicator = new RGBIndicator(hardwareMap.get(Servo.class, "rgb"));
        follower = Constants.createFollower(hardwareMap);
        pushServo_timer = new Timer();
        
        // Initialize drive control
        drive = new TeleOpDriveControl(hardwareMap);
        
        // Initialize VelocityCalculator with start pose
        velocityCalc = new VelocityCalculator(RED_AUTO_ARCHIVE.startPose, VelocityCalculator.Type.POSE);

        intake = hardwareMap.dcMotor.get("intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        pushServo = hardwareMap.get(CRServo.class, "push");

        // SET DIRECTION FOR MOTORS
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Consts.LaunchPIDF);
        pushServo.setPower(Consts.TRANSFER_DOWN_POSITION);
    }

    @Override
    public void init_loop() {
        limelight.LLUpdate();
        telemetry.addLine("=== VELOCITY TUNER ===\n");
        telemetry.addLine("Instructions:");
        telemetry.addLine("1. Drive to different distances");
        telemetry.addLine("2. Press START to record data point");
        telemetry.addLine("3. Adjust velocity with DPAD");
        telemetry.addLine("4. Test shot with LEFT BUMPER\n");
        telemetry.addData("Current Distance", "%.1f inches", getCurrentDistance());
        telemetry.addData("Angle to Target", "%.1f degrees", limelight.tx);
        telemetry.addData("Data Points Collected", tuningData.size());
        
        updateRGBIndicator();
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setStartingPose(RED_AUTO_ARCHIVE.endPose);
        follower.startTeleopDrive();
        limelight.start(0);
    }

    @Override
    public void loop() {
        follower.update();
        velocityCalc.update(follower.getPose());
        limelight.LLUpdate();
        currentDistance = getCurrentDistance();
        
        // Handle servo state machine (non-blocking)
        handleServoStateMachine();
        
        // Use old drive control
        drive.move(gamepad1);
        
        // Tuning controls
        if (gamepad1.dpad_right) {
            recordDataPoint();
        }
        if (gamepad1.dpad_left) {
            clearTuningData();
        }
        
        // Display tuning information
        double calcVelocity = velocityCalc.getVelocity();
        telemetry.addLine("=== VELOCITY TUNER ===\n");
        telemetry.addData("Distance to Goal", "%.1f inches", currentDistance);
        telemetry.addData("Calculated Velocity", "%.0f", calcVelocity);
        telemetry.addData("Current Launcher Vel", "%.0f", launcher.getVelocity());
        telemetry.addData("Angle to Target", "%.1f°", limelight.tx);
        telemetry.addData("Recording", isRecording ? "ON" : "OFF");
        telemetry.addLine();
        
        // Display collected data
        telemetry.addLine("Data Points (" + tuningData.size() + "):");
        for (int i = Math.max(0, tuningData.size() - 5); i < tuningData.size(); i++) {
            telemetry.addLine(tuningData.get(i).toString());
        }
        telemetry.addLine();
        
        // Show suggested Kotlin code
        if (tuningData.size() >= 3) {
            telemetry.addLine("=== SUGGESTED TUNING MAP ===\n");
            telemetry.addLine(generateKotlinCode());
        }
        
        updateRGBIndicator();

        // Controls
        if (oneGamepadControl) intake.setPower(gamepad1.x ? 1.0 : 0.0);
        else intake.setPower((Math.max(Math.min(gamepad2.left_stick_y * 1.1, 1), -1)));
        
        if (gamepad2.a || oneGamepadControl && gamepad1.a) {
            setVelocity(Consts.TARGET_VELOCITY_BACK_LAUNCH_ZONE);
        }
        if (gamepad2.y || oneGamepadControl && gamepad1.y) {
            setVelocity(Consts.TARGET_VELOCITY_FRONT_LAUNCH_ZONE);
        }
        // Use calculated velocity
        if (gamepad1.right_trigger > 0.5) {
            setVelocity(calcVelocity);
        }
        if (gamepad2.b || oneGamepadControl && gamepad1.b) {
            setVelocity(Consts.STOP_VELOCITY);
        }
        
        // Servo control - trigger state machine
        if ((gamepad2.left_bumper || oneGamepadControl && gamepad1.left_bumper) && servoState == ServoState.IDLE) {
            servoState = ServoState.GOING_UP;
            pushServo_timer.resetTimer();
        }
        
        // Fine tune launcher velocity
        if (gamepad2.dpad_up || oneGamepadControl && gamepad1.dpad_up) {
            setVelocity(launcher.getVelocity() + 25);
        } else if (gamepad2.dpad_down || oneGamepadControl && gamepad1.dpad_down) {
            setVelocity(launcher.getVelocity() - 25);
        }
        
        telemetry.update();
    }

    private void handleServoStateMachine() {
        switch (servoState) {
            case GOING_UP:
                // Wait until the launcher reaches target velocity
                if (launcher.getVelocity() >= targetVelocity - Consts.VELOCITY_TOLERANCE) {
                    pushServo.setPower(Consts.TRANSFER_UP_POSITION);
                    servoState = ServoState.LAUNCHING;
                    pushServo_timer.resetTimer();
                }
                break;
            case LAUNCHING:
                if (pushServo_timer.getElapsedTimeSeconds() >= Consts.SLEEP_BEFORE_TRANSFER_RESET / 1000.0) {
                    servoState = ServoState.GOING_DOWN;
                    pushServo_timer.resetTimer();
                }
                break;
            case GOING_DOWN:
                pushServo.setPower(Consts.TRANSFER_DOWN_POSITION);
                servoState = ServoState.IDLE;
                pushServo_timer.resetTimer();
                break;
            case IDLE:
            default:
                pushServo.setPower(Consts.TRANSFER_DOWN_POSITION);
                break;
        }
    }
    
    private double getCurrentDistance() {
        return velocityCalc.getDistanceToGoal(follower.getPose());
    }
    
    private void updateRGBIndicator() {
        if (limelight.isDisconnected) {
            rgbIndicator.setColor(RGBIndicator.Color.BLACK);
        } else if (limelight.getLLScore() < 1) {
            rgbIndicator.setColor(RGBIndicator.Color.GREEN);
        } else if (limelight.getLLScore() < 5) {
            rgbIndicator.setColor(RGBIndicator.Color.YELLOW);
        } else if (limelight.getLLScore() < 12.5) {
            rgbIndicator.setColor(RGBIndicator.Color.ORANGE);
        } else {
            rgbIndicator.setColor(RGBIndicator.Color.BLACK);
        }
    }
    
    private void recordDataPoint() {
        if (launcher.getVelocity() > 0) {
            boolean successful = limelight.getLLScore() < 2.0; // Success if well aimed
            TuningDataPoint point = new TuningDataPoint(
                currentDistance, 
                launcher.getVelocity(), 
                successful
            );
            tuningData.add(point);
        }
    }
    
    private void clearTuningData() {
        tuningData.clear();
    }
    
    @SuppressLint("DefaultLocale")
    private String generateKotlinCode() {
        if (tuningData.size() < 2) return "Need more data points";
        
        StringBuilder sb = new StringBuilder();
        sb.append("val VEL_MAP: Map<Double, Double> = mapOf(\n");
        
        // Sort by distance and take successful shots
        tuningData.stream()
            .filter(p -> p.successful)
            .sorted((a, b) -> Double.compare(a.distance, b.distance))
            .forEach(point -> {
                sb.append(String.format("    %.1f to %.0f,\n", 
                    point.distance, point.velocity));
            });
            
        sb.append(")");
        return sb.toString();
    }

}
