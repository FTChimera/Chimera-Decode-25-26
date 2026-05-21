/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.biobuzzTest;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This file includes a teleop (driver-controlled) file for a
 * Mecanum drivetrain. This teleop has field centric and robot centric control
 */

@TeleOp(name = "MECANUM TELEOP", group = "1: Tests")
public class MecanumTeleOp extends OpMode {

    /*
     * Enum to represent the different control types. This is used to switch between field centric and robot centric control, or tests
     * We put a method to easily switch between the different control types.
     */

    enum Mode {
        ROBOT_CENTRIC_TELEOP,
        FIELD_CENTRIC_TELEOP,
        MOTOR_DIRECTION_TEST;

        public Mode next(int amt) {
            return values()[(ordinal() + amt) % values().length];
        }
    }

    // Declare OpMode members.
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private IMU imu = null;
    Mode teleopMode = Mode.ROBOT_CENTRIC_TELEOP;

    /* Constants */
    private final double DRIVE_POWER_SCALAR = 0.8; // Scalar for drive power to prevent overpowering the motors, and easier driving.
    private final double MOTOR_DIRECTION_POWER = 0.4; // power for motor direction test mode to see clearly if the motors are going in the right direction or not.
    private final double STRAFE_POWER_COMPENSATION = 1.1; // Compensation for imperfect strafing on most mecanum drives. This is multiplied by the strafing power to make the robot strafe more effectively.
    private final double JOYSTICK_DEADZONE = 0.05; // Deadzone for joysticks to prevent drift. Inputs within the deadzone are set to 0.
    private final double JOYSTICK_CURVE_EXPONENT = 1.5; // Exponent for joystick curve. This makes the controls less sensitive at low inputs for finer control, while still allowing full power at max input.
    private final double TRIGGER_THRESHOLD = 0.4; // Threshold for triggers to prevent accidental activation. Inputs below the threshold are set to 0, and inputs above the threshold are set to 1 to make it act like a button.

    /*
     * Code to run ONCE when the driver hits INIT
     * This initializes hardware
     */
    @Override
    public void init() {

        /*
         * Initialize the drive motor variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftFront = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        rightFront = hardwareMap.get(DcMotor.class, "frontRightMotor");
        leftBack = hardwareMap.get(DcMotor.class, "backLeftMotor");
        rightBack = hardwareMap.get(DcMotor.class, "backRightMotor");

        /*
         * To drive forward, most robots need the motors on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Initialize the IMU. To do that we have to tell it which direction it is facing.
         * We call hardwareMap.get
         * To initialize IMU parameters, we use the logo and usb facing directions to tell it
         * what direction it is facing.
         */
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(imuParameters);
        imu.resetYaw();
        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized Hardware Components");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     * This runs the selection process for field centric vs robot centric control.
     */
    @Override
    public void init_loop() {
        /*
         * Here we use the dpad Left/Right to switch between teleop modes.
         * The selection is shown on the driver station via telemetry.
         */
        if (gamepad1.dpadLeftWasPressed()) {
            teleopMode = teleopMode.next(1);
        }
        if (gamepad1.dpadRightWasPressed()) {
            teleopMode = teleopMode.next(-1);
        }

        telemetry.addData("TeleOp Mode", teleopMode);
        telemetry.addData("Instructions", "Press DPAD Left/Right to switch modes");
        telemetry.addData("IMU Yaw", "(%.3f) degrees", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     * Right now, this method is blank.
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     * Here we call our main teleop code to control the robot. We also show telemetry data on the driver station.
     */
    @Override
    public void loop() {

        /*
         * Here, lets add an IMU reset option
         * This is useful in field centric, to reset the heading so that the robot has a more accurate heading reference.
         * It is a good idea to keep it as a global method in loop() and not just for field centric
         * Here we use Back to reset yaw. This is also the same as share in PS4/PS5 controllers.
         */
        if (gamepad1.backWasPressed()) {
            imu.resetYaw();
        }

        /*
         * Here we use the left stick for translation and the right stick for rotation. The controls are squared to allow for finer control at low speeds.
         * We also negate the inputs because by default, pushing the stick forward gives a negative value, and we want that to correspond to positive power.
         * The method of passing in the inputs depends on the teleop mode, which is selected in the init_loop method.
         * We keep these global variables through the method for telemetry
         */

        double forward=0, strafe=0, rotate=0;
        double leftStickX, leftStickY, rightStickX; // We declare these here to avoid redeclaring them in each case of the switch statement, since they are used in multiple cases.

        /*
         * Here we implement the code for each teleop mode to set drive powers
         */
        switch (teleopMode) {
            case ROBOT_CENTRIC_TELEOP:
                // Implement Joystick deadzone and curve
                leftStickX = Math.abs(gamepad1.left_stick_x) > JOYSTICK_DEADZONE ? gamepad1.left_stick_x : 0;
                leftStickY = Math.abs(gamepad1.left_stick_y) > JOYSTICK_DEADZONE ? gamepad1.left_stick_y : 0;
                rightStickX = Math.abs(gamepad1.right_stick_x) > JOYSTICK_DEADZONE ? gamepad1.right_stick_x : 0;
                leftStickX = Math.copySign(Math.pow(Math.abs(leftStickX), JOYSTICK_CURVE_EXPONENT), leftStickX);
                leftStickY = Math.copySign(Math.pow(Math.abs(leftStickY), JOYSTICK_CURVE_EXPONENT), leftStickY);
                rightStickX = Math.copySign(Math.pow(Math.abs(rightStickX), JOYSTICK_CURVE_EXPONENT), rightStickX);
                forward = -leftStickX;
                strafe = leftStickY * STRAFE_POWER_COMPENSATION;
                rotate = rightStickX;
                break;
            case FIELD_CENTRIC_TELEOP:
                // Implement Joystick deadzone and curve
                leftStickX = Math.abs(gamepad1.left_stick_x) > JOYSTICK_DEADZONE ? gamepad1.left_stick_x : 0;
                leftStickY = Math.abs(gamepad1.left_stick_y) > JOYSTICK_DEADZONE ? gamepad1.left_stick_y : 0;
                rightStickX = Math.abs(gamepad1.right_stick_x) > JOYSTICK_DEADZONE ? gamepad1.right_stick_x : 0;
                leftStickX = Math.copySign(Math.pow(Math.abs(leftStickX), JOYSTICK_CURVE_EXPONENT), leftStickX);
                leftStickY = Math.copySign(Math.pow(Math.abs(leftStickY), JOYSTICK_CURVE_EXPONENT), leftStickY);
                rightStickX = Math.copySign(Math.pow(Math.abs(rightStickX), JOYSTICK_CURVE_EXPONENT), rightStickX);
                forward = -leftStickX;
                strafe = leftStickY * STRAFE_POWER_COMPENSATION;
                rotate = rightStickX;
                // For field centric, we need to rotate the inputs by the direction of the robot to turn them into robot centric inputs
                double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double cosA = Math.cos(robotHeading);
                double sinA = Math.sin(robotHeading);
                forward = forward * cosA + strafe * sinA;
                strafe = -forward * sinA + strafe * cosA;
                break;
            case MOTOR_DIRECTION_TEST:
                // In this mode, the motors move forward based on gamepad right trigger. This allows you to easily see if any motors are reversed or not, and adjust the motor directions accordingly.
                forward = (gamepad1.right_trigger >= TRIGGER_THRESHOLD ? 1 : 0) * MOTOR_DIRECTION_POWER;
                strafe = 0;
                rotate = 0;
                if (gamepad1.aWasPressed()) {
                    leftFront.setDirection(leftFront.getDirection().inverted());
                }
                if (gamepad1.bWasPressed()) {
                    rightFront.setDirection(rightFront.getDirection().inverted());
                }
                if (gamepad1.xWasPressed()) {
                    leftBack.setDirection(leftBack.getDirection().inverted());
                }
                if (gamepad1.yWasPressed()) {
                    rightBack.setDirection(rightBack.getDirection().inverted());
                }

                telemetry.addLine("Use A to invert left front motor direction");
                telemetry.addLine("Use B to invert right front motor direction");
                telemetry.addLine("Use X to invert left back motor direction");
                telemetry.addLine("Use Y to invert right back motor direction");
                break;
        }

        // Now, calculate drive powers based on the forward, strafe, and rotate values, and set the motor powers.
        double[] motorPowers = new double[4];
        motorPowers[0] = (forward + strafe + rotate) * DRIVE_POWER_SCALAR; // Front Left
        motorPowers[1] = (forward - strafe - rotate) * DRIVE_POWER_SCALAR; // Front Right
        motorPowers[2] = (forward - strafe + rotate) * DRIVE_POWER_SCALAR; // Back Left
        motorPowers[3] = (forward + strafe - rotate) * DRIVE_POWER_SCALAR; // Back Right

        // Now, set powers to motors
        leftFront.setPower(motorPowers[0]);
        rightFront.setPower(motorPowers[1]);
        leftBack.setPower(motorPowers[2]);
        rightBack.setPower(motorPowers[3]);

        /*
         * Show motor powers on the Driver Station via telemetry.
         */
        telemetry.addData("TeleOp Mode", teleopMode);
        telemetry.addData("Drive Power", "forward (%.2f), strafe (%.2f), rotate (%.2f)", forward, strafe, rotate);
        telemetry.addData("IMU Yaw", "(%.3f) degrees", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addLine("Use Back/Share to reset IMU Yaw");
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     * Here we stop the motors to prevent the robot from moving after teleop ends.
     */
    @Override
    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}