package org.firstinspires.ftc.teamcode2.Systems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

public class AutoAlignSystem {

    // Test class of how this works
    // todo FINISH THIS CLASS FOR LIMELIGHT


    // PEDRO STUFF
    private Pose currentPose, newPose;
    double Posex, Posey, diffX, diffY;
    double angle;
    private Consts.AllianceColor currentGoal;

    private Follower follower;
    Path autoAlignmentPath;
    Pose GoalCoordinates;

    // LIMELIGHT STUFF
    private LimelightSystem limelight;
    private PIDFController limelightPIDF;

    private final double TxOffset = 0;
    double tx, tid;
    private DcMotor[] drivingMotors; // for auto alignment turning

    boolean areWeUsingPedro;


    public AutoAlignSystem(Consts.AllianceColor allianceColor) {
        currentGoal = allianceColor;
        if (currentGoal == Consts.AllianceColor.RED) {
            GoalCoordinates = new Pose(
                    0,144
            ); // RED GOAL COORDINATES
        } else {
            GoalCoordinates = new Pose(
                    144, 144
            ); // BLUE GOAL COORDINATES
        }
    }

    public void PedroSetUp(Follower follower1) {
        follower = follower1;
        areWeUsingPedro = true;
    }

    public void LimelightSetUp(LimelightSystem limelightSystem, DcMotor[] driveMotors) {
        limelight = limelightSystem;
        drivingMotors = driveMotors;
        limelightPIDF = new PIDFController(
            Consts.LimelightAutoAlignmentTurning,
            Consts.LIMELIGHT_PIDF_MIN_OUTPUT, 
            Consts.LIMELIGHT_PIDF_MAX_OUTPUT, 
            Consts.LIMELIGHT_PIDF_INTEGRAL_LIMIT
        );
        areWeUsingPedro = false;
    }

    // main function to auto align to goal
    public void turnAutoAlign(double deltaTime) {
        if (areWeUsingPedro) turnAutoAlignPedro(); else turnAutoAlignLimelight(deltaTime);
    }

    public void turnAutoAlign() {
        if (areWeUsingPedro) turnAutoAlignPedro(); else turnAutoAlignLimelight(0.02);
    }

    // 2 functions
    private void turnAutoAlignPedro() {
        currentPose = follower.getPose();
        Posex = currentPose.getX();
        Posey = currentPose.getY();

        // Find angle using tan inverse
        // Calculate signed differences to get correct direction

        diffX = GoalCoordinates.getX() - Posex;
        diffY = GoalCoordinates.getY() - Posey;

        // tan theta = opposite divided by adjacent = y/x
        // atan2 handles all quadrants correctly

        angle = Math.atan2(diffY, diffX);

        // Use Pedro's turn functionality for pure rotation
        // This is more efficient than creating a path with same coordinates
        follower.turnToHeading(angle);
    }

    private void turnAutoAlignLimelight(double dt) {
        // Use the tx value from the already updated limelight system
        tx = limelight.tx;
        tid = limelight.tid;
        
        // Check if we have a valid target
        if (limelight.isDisconnected || tx == 0) {
            // No valid target, stop rotation
            if (areWeUsingPedro) {
                follower.setTeleOpDrive(0, 0, 0, true);
            } else {
                // Stop all drive motors
                for (DcMotor motor : drivingMotors) {
                    motor.setPower(0);
                }
            }
            return;
        }
        
        // Verify tag ID matches alliance color for DECODE season (24 for red goal, 20 for blue goal)
        boolean correctTag = false;
        if (currentGoal == Consts.AllianceColor.RED && tid == 24) {
            correctTag = true;
        } else if (currentGoal == Consts.AllianceColor.BLUE && tid == 20) {
            correctTag = true;
        }
        
        if (!correctTag) {
            return; // Wrong target, don't align
        }
        
        // Apply offset and calculate rotation command using PID with dt from TeleOp
        double error = tx + TxOffset;
        double rotationCmd = limelightPIDF.update(error, dt);
        
        // Apply rotation command
        if (areWeUsingPedro) {
            follower.setTeleOpDrive(0, 0, rotationCmd, true);
        } else {
            // Manual motor control for mecanum drive
            // Assuming standard mecanum wheel configuration
            double leftFrontPower = rotationCmd;
            double leftBackPower = rotationCmd;
            double rightFrontPower = -rotationCmd;
            double rightBackPower = -rotationCmd;
            
            // Apply power to motors (assuming standard order: lf, lb, rf, rb)
            if (drivingMotors.length >= 4) {
                drivingMotors[0].setPower(leftFrontPower);
                drivingMotors[1].setPower(leftBackPower);
                drivingMotors[2].setPower(rightFrontPower);
                drivingMotors[3].setPower(rightBackPower);
            }
        }
    }
}
