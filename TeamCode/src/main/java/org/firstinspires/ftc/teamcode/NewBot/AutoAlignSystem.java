package org.firstinspires.ftc.teamcode.NewBot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;


@SuppressWarnings("SpellCheckingInspection")
public class AutoAlignSystem {
    // PEDRO STUFF
    private Pose currentPose, newPose;
    double Posex, Posey, diffX, diffY, PoseAngle;
    double angle;
    private Constants.AllianceColor currentGoal;

    private Follower follower;
    Path autoAlignmentPath;
    Pose GoalCoordinates;

    // LIMELIGHT STUFF
    private LimelightSystem limelight;
    private PIDFController limelightPIDF;

    private double TxOffset = 0;
    double tx;
    int tid;
    private DcMotor[] drivingMotors; // for auto alignment turning

    boolean drivingMotorsUsed;
    boolean areWeUsingPedro;
    // Controller for compensation when robot is pushed.
    public static PIDFController autoAlign_pushed = new PIDFController(
            Constants.AUTOALIGN_PUSHED, Constants.LIMELIGHT_PIDF_MIN_OUTPUT, Constants.LIMELIGHT_PIDF_MAX_OUTPUT, Constants.LIMELIGHT_PIDF_INTEGRAL_LIMIT
    ); // Use the same min-max-integralLimit as Limelight as the output goes to the same thing
    // need to use pedro pathing pose to measure velocity of robot
    public static PIDFController autoAlignPedro = new PIDFController(
            Constants.PedroAutoAlignmentTurning, Constants.LIMELIGHT_PIDF_MIN_OUTPUT, Constants.LIMELIGHT_PIDF_MAX_OUTPUT, Constants.LIMELIGHT_PIDF_INTEGRAL_LIMIT
    );
    public void reset() {
        limelightPIDF.reset();
        autoAlign_pushed.reset();
        if (areWeUsingPedro) follower.startTeleopDrive();
    }

    public AutoAlignSystem(Constants.AllianceColor allianceColor){
        currentGoal = allianceColor;
        if (currentGoal == Constants.AllianceColor.RED) {
            GoalCoordinates = Constants.RED_GOAL;
        } else {
           GoalCoordinates = Constants.BLUE_GOAL;
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
                Constants.LimelightAutoAlignmentTurning,
                Constants.LIMELIGHT_PIDF_MIN_OUTPUT,
                Constants.LIMELIGHT_PIDF_MAX_OUTPUT,
                Constants.LIMELIGHT_PIDF_INTEGRAL_LIMIT
        );
        areWeUsingPedro = false;
        drivingMotorsUsed = true;
    }
    public void LimelightSetUp(LimelightSystem limelightSystem) {
        this.LimelightSetUp(limelightSystem, null);
        drivingMotorsUsed = false;
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
        follower.turnTo(angle);
    }

    public void turnAutoAlignLimelight(double dt) {
        if (!drivingMotorsUsed) return;
        tid = currentGoal.getTagID();
        if (!LLCanSeeGoal() && !(limelight.isTagInFiducialResults(tid))) return;
        // Use the tx value from the already updated limelight system
        tx = limelight.getResultForTag(tid).getTargetXDegrees();

        // Check if we have a valid target
        if (limelight.isDisconnected || tx == 0) {
            // No valid target, stop rotation
            // Stop all drive motors
            for (DcMotor motor : drivingMotors) {
                motor.setPower(0);
            }
            return;
        }

        // Verify tag ID matches alliance color for DECODE season (24 for red goal, 20 for blue goal)

        double rotationCmd = getTurningPowerLimelightWithSuppliedTX(dt, tx);
        // Apply rotation command

        // Manual motor control for mecanum drive
        // Assuming standard mecanum wheel configuration
        double leftFrontPower = rotationCmd;
        double leftBackPower = rotationCmd;
        double rightFrontPower = -rotationCmd;
        double rightBackPower = -rotationCmd;

        // Apply power to motors (assuming standard order: lf, lb, rf, rb)
        if (drivingMotors.length == 4) {
            drivingMotors[0].setPower(leftFrontPower);
            drivingMotors[1].setPower(leftBackPower);
            drivingMotors[2].setPower(rightFrontPower);
            drivingMotors[3].setPower(rightBackPower);
        }

    }
    public double getTurningPowerLimelight(double dt) {
        // Use the tx value from the already updated limelight system
        // Verify tag ID matches alliance color for DECODE season (24 for red goal, 20 for blue goal)

        if (!LLCanSeeGoal()) {
            return 0; // Wrong target, don't align
        }

        // Guard against PID controller not initialized
        if (limelightPIDF == null) {
            return 0;
        }

        double error = tx + TxOffset;
        double rotationCmd = limelightPIDF.updatePIDF(error, dt);

        if (Math.abs(rotationCmd) < 0.1) {
            rotationCmd = Math.copySign(0.1, rotationCmd);
        }
        // return rotation command for use in TeleOp
        return rotationCmd;
    }
    public boolean isErrorAtTolerance() {
        limelightPIDF.setTolerance(1);
        boolean result = limelightPIDF.atSetpoint(limelightPIDF.getLastError());
        return limelightPIDF.getLastError() != 0.0 && result;
    }
    public double getTurningPowerPedro(Pose currentPose, double dt) {
        // Use the tx value from the already updated limelight system
        // Verify tag ID matches alliance color for DECODE season (24 for red goal, 20 for blue goal)

        if (!LLCanSeeGoal()) {
            return 0; // Wrong target, don't align
        }

        // Guard against PID controller not initialized
        if (limelightPIDF == null) {
            return 0;
        }
        double correctHeading = Math.atan2(
                GoalCoordinates.getY() - currentPose.getY(),
                GoalCoordinates.getX() - currentPose.getX());

        double headingError = correctHeading - currentPose.getHeading();
        double rotationCmd = limelightPIDF.updatePIDF(headingError, dt);
        return rotationCmd;
    }

    public boolean LLCanSeeGoal() {
        // Verify tag ID matches alliance color for DECODE season (24 for red goal, 20 for blue goal)
        boolean correctTag = false;

        // Prefer checking reported tid first (simple and fast), then fallback to fiducial results
        if (currentGoal == Constants.AllianceColor.RED) {
            if (limelight.tid == 24) {
                correctTag = true;
                tid = 24;
                // prefer the detailed result if available
                if (limelight.isTagInFiducialResults(24)) {
                    tx = limelight.getResultForTag(24).getTargetXDegrees();
                } else {
                    tx = limelight.tx; // fallback
                }
            } else if (limelight.isTagInFiducialResults(24)) {
                correctTag = true;
                tid = 24;
                tx = limelight.getResultForTag(24).getTargetXDegrees();
            }
        } else if (currentGoal == Constants.AllianceColor.BLUE) {
            if (limelight.tid == 20) {
                correctTag = true;
                tid = 20;
                if (limelight.isTagInFiducialResults(20)) {
                    tx = limelight.getResultForTag(20).getTargetXDegrees();
                } else {
                    tx = limelight.tx;
                }
            } else if (limelight.isTagInFiducialResults(20)) {
                correctTag = true;
                tid = 20;
                tx = limelight.getResultForTag(20).getTargetXDegrees();
            }
        }

        return correctTag;
    }

    public double getTurningPowerLimelightWithSuppliedTX(double deltaTime, double targetX) {
        // Apply offset and calculate rotation command using PID with dt from TeleOp
        double error = targetX + TxOffset;
        double rotationCmd = limelightPIDF.updatePIDF(error, deltaTime);

        // return rotation command for use in TeleOp
        return rotationCmd;
    }


    // NORMALIZE HEADING IN RADIANS
    public static double normalizeHeading(double heading) {
        while (heading > Math.PI) {
            heading -= 2 * Math.PI;
        }
        while (heading < -Math.PI) {
            heading += 2 * Math.PI;
        }
        return heading;
    }
    public double getTurningPowerPose(Pose robotPose, Vector robotVelocity, double deltaTime, boolean shouldCompensateAutoAlign) {
        Posex = robotPose.getX();
        Posey = robotPose.getY();
        PoseAngle = robotPose.getHeading();

        // Find angle using tan inverse
        // Calculate signed differences to get correct direction

        diffX = GoalCoordinates.getX() - Posex;
        diffY = GoalCoordinates.getY() - Posey;

        // tan theta = opposite divided by adjacent = y/x
        // atan2 handles all quadrants correctly

        angle = Math.atan2(diffY, diffX);

        // normalize angle
        double turnTo = angle - PoseAngle;
        turnTo = Math.toDegrees(normalizeHeading(turnTo)); // turn to degrees

        // Compensation logic
        if (shouldCompensateAutoAlign) {
            double direction = normalizeHeading(angle - robotVelocity.getTheta());
            turnTo += autoAlign_pushed.updatePIDF(robotVelocity.getMagnitude() * Math.signum(direction), deltaTime);
        }


        return autoAlignPedro.updatePIDF(turnTo, deltaTime);
    }
}
