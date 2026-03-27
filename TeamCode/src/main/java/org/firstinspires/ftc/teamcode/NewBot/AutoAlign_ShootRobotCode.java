package org.firstinspires.ftc.teamcode.NewBot;

import static org.firstinspires.ftc.teamcode.Systems.RGBIndicator.GREEN_PWM;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Systems.LimelightSystem;
import org.firstinspires.ftc.teamcode.Systems.RGBIndicator;

@TeleOp(name = "Robot Code Shoot+ auto align", group = "0:TeleOp")
public class AutoAlign_ShootRobotCode extends LinearOpMode {
    public LimelightSystem limelightSystem;
    public RGBIndicator indicator;
    public DcMotor[] driveMotors;
    public AutoAlignSystem autoAlignSystem;
    double deltaTime = 0.02; // For now: (derivative component not enabled)


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get(Constants.pedroMecanumDriveConstants.leftFrontMotorName);
        DcMotor backLeftMotor = hardwareMap.dcMotor.get(Constants.pedroMecanumDriveConstants.leftRearMotorName);
        DcMotor frontRightMotor = hardwareMap.dcMotor.get(Constants.pedroMecanumDriveConstants.rightFrontMotorName);
        DcMotor backRightMotor = hardwareMap.dcMotor.get(Constants.pedroMecanumDriveConstants.rightRearMotorName);

        // Using DcMotorEx instead of DcMotor to use PID controller
        DcMotorEx OuttakeMotor = hardwareMap.get(DcMotorEx.class,"OuttakeMotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor transferMotor = hardwareMap.dcMotor.get("transferMotor");
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        driveMotors = new DcMotor[]{
                frontLeftMotor,frontRightMotor,backLeftMotor,backRightMotor
        };
        OuttakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        OuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        OuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        OuttakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Constants.LaunchPIDF);

        autoAlignSystem = new AutoAlignSystem(Constants.AllianceColor.RED);

        limelightSystem = new LimelightSystem(hardwareMap);
        indicator = new RGBIndicator(hardwareMap);
        autoAlignSystem.LimelightSetUp(limelightSystem);
        limelightSystem.start();
        while (opModeInInit()) {
            limelightSystem.LLUpdate();
            indicator.updateUsingLL(limelightSystem);
            updateAutoAlign();
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        double distance = 140;
        do {
            limelightSystem.LLUpdate();
            indicator.updateUsingLL(limelightSystem);
            LLResultTypes.FiducialResult fiducialResult = limelightSystem.getResultForTag(Constants.AllianceColor.RED.getTagID());
            if (!(fiducialResult==null)) {
                distance = limelightSystem.calculateDistance(fiducialResult);
                if (distance == 0) {
                    distance = limelightSystem.dist; // SAFETY CHECK
                }
            }
            updateAutoAlign();
            OuttakeMotor.setVelocity(VelocityCalculator.NEWBOT.calculateVelocity(distance));
            transferMotor.setPower(Constants.TRANSFER_UP_POSITION);
            intakeMotor.setPower(1);
        } while (opModeIsActive());
    }

    public void updateAutoAlign() {
        double rotationCmd = -autoAlignSystem.getTurningPowerLimelight(deltaTime);
        if (indicator.getPWM() == GREEN_PWM) {rotationCmd = 0;}
        for (int i = 0; i < 3; i++) driveMotors[i].setPower(rotationCmd);
    }
}
