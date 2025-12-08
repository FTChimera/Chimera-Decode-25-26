package org.firstinspires.ftc.teamcode.Vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Systems.Consts;
import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode.Systems.RGBIndicator;
import org.firstinspires.ftc.teamcode.pedroAuto.Constants;

@TeleOp
public class LLOrientation extends LinearOpMode {
    enum SelectionChange{UP, DOWN}
    public static Consts.AllianceColor allianceColor = Consts.AllianceColor.RED;
    public String[] options = {"April Heading Rotation", "Distance Test", "April Tag Test", "RGB Indicator Test", "PIDF Tuning Test", "WHITE LIGHT INDICATOR"};
    public int mode;
    private Follower follower;
    public LimelightSystem.ChimeraLL limelight = new LimelightSystem.ChimeraLL();
    void renderSelectMode(int selected) {
        telemetry.addData("Button to change between testing modes", "Gamepad 2 Dpad up/down");
        telemetry.addData("Button to select testing mode", "Gamepad2 A");
        telemetry.addData("Options","");
        for (int i = 0; i < options.length; i++) {
            telemetry.addLine((i==selected?"> ":"") + options[i]);
        }
        telemetry.update();
    }
    public int changeSelection(SelectionChange direction, int select, int maxSelect) {
        if (direction==SelectionChange.UP) {
            if (select==0) return maxSelect; else return (select-1);
        } else if (direction==SelectionChange.DOWN) {
            if (select==maxSelect) return 0; else return (select+1);
        }
        return -1;
    }
    public double findAngleToRotate() {
        // return allianceColor==Consts.AllianceColor.RED? Math.atan2( (Consts.Y_Coordinate_Red_Goal - curPose.getY()), (Consts.X_Coordinate_Red_Goal - curPose.getX()) ):Math.atan2( (Consts.Y_Coordinate_Blue_Goal - curPose.getY()), (Consts.X_Coordinate_Blue_Goal - curPose.getX()) );
        return Math.toRadians(limelight.tx);
    }
    public Path getPath(double amt) {
        Pose pose = follower.getPose();
        double heading = pose.getHeading();
        double RobotCentric_X = allianceColor==Consts.AllianceColor.RED?Math.cos(heading):Math.sin(heading);
        double RobotCentric_Y = allianceColor==Consts.AllianceColor.RED?-Math.sin(heading):Math.cos(heading);
        Pose endPose = new Pose(pose.getX() + RobotCentric_X*amt, pose.getY() + RobotCentric_Y*amt, heading);
        Path path = new Path(new BezierLine(pose, endPose));
        path.setConstantHeadingInterpolation(heading);
        return path;
    }
    public void rotate(double angle_Radians) {follower.turn(Math.abs(angle_Radians), angle_Radians/Math.abs(angle_Radians)==-1);follower.update();}

    public double getLLScore() {
        return Math.abs(limelight.tx) + Math.abs(limelight.dist);
    }
    @Override
    public void runOpMode() {
        RGBIndicator indicator = new RGBIndicator(hardwareMap.get(Servo.class, "rgb"));
        follower = Constants.createFollower(hardwareMap);int pipe = 0; // int pipe = allianceColor==Consts.AllianceColor.RED?4:5
        limelight.setDevice(hardwareMap.get(Limelight3A.class, "limelight"));
        int selected=0;
        while (opModeInInit()) {
            renderSelectMode(selected);
            if (gamepad2.dpad_up){selected=changeSelection(SelectionChange.UP,selected,options.length-1);}
            if (gamepad2.dpad_down){selected=changeSelection(SelectionChange.DOWN,selected,options.length-1);}
            if (gamepad2.a) {telemetry.addData("Option Selected",options[selected]);telemetry.update();break;}
        }
        double[] maxVelocity = {0,0,0,0};
        double[] currentVelocity = {0,0,0,0};
        waitForStart();limelight.startLLWithPipeline(pipe);follower.startTeleopDrive();

        while (opModeIsActive()) {
            limelight.LLUpdate();
            switch (selected) {
                case 0: // April Heading Rotation
                    telemetry.addData("Tx (radians)", Math.toRadians(limelight.tx));
                    telemetry.addData("Tx (degrees)", limelight.tx);
                    telemetry.addData("Gamepad2.a", "not pressed");
                    if (gamepad2.a) {
                        int step = 0;
                        if (gamepad2.dpad_right) {step=step+1;}
                        telemetry.addData("Gamepad2.a", "pressed");
                        // FIRST, rotate to face the goal
                        if (step==0) {
                            rotate(findAngleToRotate() * Consts.LAUNCHER_GOALTAG_ANGLE_SCALE);
                        }
                        // THEN, you need to move so that the distance is good for launching.
                        if (step==1){
                            follower.followPath(getPath(limelight.dist));
                            follower.update();
                        }
                    } else {
                        follower.update();
                    }

                    telemetry.update();
                    break;
                case 1: // Distance Test
                    break;
                case 2: // April Tag Test
                    telemetry.addLine("TX AND TY ARE IN DEGREES");
                    telemetry.addData("TX",limelight.tx);
                    telemetry.addData("TY",limelight.ty);
                    telemetry.addData("TA",limelight.ta);
                    telemetry.addData("ID",limelight.tid);
                    telemetry.addData("DIST",limelight.dist);
                    break;
                case 3: // RGB Indicator Test
                    telemetry.addData("LLScore", getLLScore());
                    if (getLLScore() < 5) {
                        telemetry.addLine("Indicator: GREEN");
                    } else if (getLLScore() < 10) {
                        telemetry.addLine("Indicator: ORANGE");
                    } else {
                        telemetry.addLine("Indicator: OFF");
                    }
                    break;
                case 4: // PIDF Tuning Test
                    double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                    double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                    double rx = gamepad1.right_stick_x;
                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    double[] motorPowerValues = {0.85 *(y + x + rx) / denominator,0.85 *(y - x + rx) / denominator,0.85 *(y - x - rx) / denominator,0.85* (y + x - rx) / denominator};
                    for (int i = 0; i<4; i++) {
                        hardwareMap.get(DcMotorEx.class, Consts.DRIVE_MOTOR_NAMES[i]).setPower(motorPowerValues[i]);
                        currentVelocity[i] = hardwareMap.get(DcMotorEx.class, Consts.DRIVE_MOTOR_NAMES[i]).getVelocity();
                        if (maxVelocity[i] < currentVelocity[i]) maxVelocity[i] = currentVelocity[i];
                        telemetry.addData("Current Motor", Consts.DRIVE_MOTOR_NAMES[i]);
                        telemetry.addData("Current Velocity", currentVelocity[i]);
                        telemetry.addData("Max Velocity", maxVelocity[i]);
                        telemetry.update();
                    }
                    break;
                case 5: // WHITE LIGHT INDICATOR
                    indicator.setColor(RGBIndicator.Color.WHITE);
                    telemetry.update();
                    break;
            }

        }
    }
}
