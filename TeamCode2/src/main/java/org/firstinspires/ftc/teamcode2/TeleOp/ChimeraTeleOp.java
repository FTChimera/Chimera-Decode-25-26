package org.firstinspires.ftc.teamcode2.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode2.Systems.Consts;
import org.firstinspires.ftc.teamcode2.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode2.Systems.RGBIndicator;
import org.firstinspires.ftc.teamcode2.Systems.TeleOpDriveControl;

@TeleOp(name="ChimeraTeleOp", group="AbsolutePriority")
public class ChimeraTeleOp extends OpMode {
    TeleOpDriveControl drive;
    DcMotor intake;DcMotorEx launcher;Servo pushServo;
    LimelightSystem limelight;
    Consts.AllianceColor allianceColor;boolean oneGamepadControl = false;
    RGBIndicator rgbIndicator;

    @Override
    public void init() {
        limelight = new LimelightSystem(hardwareMap);
        rgbIndicator = new RGBIndicator(hardwareMap.get(Servo.class, "rgb"));
        allianceColor = Consts.AllianceColor.RED;

        drive = new TeleOpDriveControl(hardwareMap);
        intake = hardwareMap.dcMotor.get("intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        pushServo = hardwareMap.servo.get("push");

        // SET DIRECTION FOR MOTORS
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Consts.LaunchPIDF);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Alliance Color (press Bumpers to switch)", allianceColor);
        telemetry.addData("One Gamepad control (press A to switch)", oneGamepadControl);
        if (gamepad1.right_bumper) allianceColor = Consts.AllianceColor.BLUE; rgbIndicator.setColor(RGBIndicator.Color.BLUE);
        if (gamepad1.left_bumper) allianceColor = Consts.AllianceColor.RED; rgbIndicator.setColor(RGBIndicator.Color.RED);
        if (gamepad1.aWasPressed()) oneGamepadControl = !oneGamepadControl;
        telemetry.update();
    }

    @Override
    public void start() {
        // run pedro follower set starting pose
        limelight.start(0);
    }

    @Override
    public void loop() {
        limelight.LLUpdate();
        telemetry.addData("Angle Degrees", limelight.tx); // LLScore is negative/positive
        telemetry.addData("Launcher Velocity", launcher.getVelocity());
        if (limelight.isDisconnected) {
            rgbIndicator.setColor(RGBIndicator.Color.BLACK); // Limelight not looking at target
        }
        else if (limelight.getLLScore() < 1.5) {
            // GREEN
            rgbIndicator.setColor(RGBIndicator.Color.GREEN);
        } else if (limelight.getLLScore() < 5) {
            // YELLOW
            rgbIndicator.setColor(RGBIndicator.Color.YELLOW);
        } else if (limelight.getLLScore() < 12.5) {
            // ORANGE
            rgbIndicator.setColor(RGBIndicator.Color.ORANGE);
        } else {
            // OFF
            rgbIndicator.setColor(RGBIndicator.Color.BLACK);
        }

        drive.move(gamepad1, 1.0); // TODO: use Pedro Pathing drive control
        // Maybe add button to go to fine-tuned launch zone positions for front and back
        if (oneGamepadControl) intake.setPower(gamepad1.x?1.0:0.0);
        else intake.setPower((Math.max(Math.min(gamepad2.left_stick_y *1.1,1),-1))); //Counteract imperfect intake power
        if (gamepad2.a || oneGamepadControl&&gamepad1.a) {
            launcher.setVelocity(Consts.MIN_VELOCITY_BACK_LAUNCH_ZONE);
            launcher.setVelocity(Consts.TARGET_VELOCITY_BACK_LAUNCH_ZONE);
        }
        if (gamepad2.y || oneGamepadControl&&gamepad1.y) {
            launcher.setVelocity(Consts.MIN_VELOCITY_FRONT_LAUNCH_ZONE);
            launcher.setVelocity(Consts.TARGET_VELOCITY_FRONT_LAUNCH_ZONE);
        }
        if (gamepad2.b || oneGamepadControl&&gamepad1.b) {
            launcher.setVelocity(Consts.STOP_VELOCITY);
        }
        if (gamepad2.right_bumper || oneGamepadControl&&gamepad1.right_bumper) {
            pushServo.setPosition(Consts.SERVO_UP_POSITION);
        } else {
            pushServo.setPosition(Consts.SERVO_DOWN_POSITION);
        }
        if (gamepad2.left_bumper || oneGamepadControl&&gamepad1.left_bumper) {
            runPushServoOnce();
        }
    }

    private void runPushServoOnce() {
        pushServo.setPosition(Consts.SERVO_UP_POSITION);
        // Sleep for some time to allow the servo to reach the down position
        pushServo.setPosition(Consts.SERVO_DOWN_POSITION);
    }

}
