package org.firstinspires.ftc.teamcode;
import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LauncherSubsystem {

    private DcMotorEx rightOutakeMotor, leftOutakeMotor;
    private Servo pushServo;
    // declaring our PIDF tuning values
    final double Kp = 300;
    final double Ki = 0.0;
    final double Kd = 0.0;
    final double Kf = 10;
    final double TARGET_VELOCITY_BACK_LAUNCH_ZONE = 1150;// Set target velocity from back launch zone
    final double TARGET_VELOCITY_FRONT_LAUNCH_ZONE = 250;// Set target velocity from front launch zone
    final double MIN_VELOCITY_BACK_LAUNCH_ZONE = 200;// Set target velocity from back launch zone
    final double MIN_VELOCITY_FRONT_LAUNCH_ZONE = 50;// Set target velocity from back launch zone
    final double STOP_VELOCITY = 0; // Set target velocity- in RPM(e.g., 3000 RPM)
    final int SERVO_LAUNCH_POSITION = 0;
    final int SERVO_REST_POSITION = 1;
    final int SLEEP_BEFORE_RESET_SERVO_POSITION = 600;
    double  setTargetVelocity = 0;
    double setMinVelocity = 0;


    public LauncherSubsystem(HardwareMap hardwareMap) {
        DcMotorEx rightOutakeMotor = hardwareMap.get(DcMotorEx.class,"OutakeMotorRight");
        DcMotorEx leftOutakeMotor = hardwareMap.get(DcMotorEx.class,"OutakeMotorLeft");
        Servo pushServo = hardwareMap.servo.get("pushServo");
        rightOutakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftOutakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftOutakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOutakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOutakeMotor.setZeroPowerBehavior(BRAKE);
        leftOutakeMotor.setZeroPowerBehavior(BRAKE);

        leftOutakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Kp, Ki, Kd, Kf));
        rightOutakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Kp, Ki, Kd, Kf));

    }

    public void runOutake()
    {
        setMinVelocity = MIN_VELOCITY_FRONT_LAUNCH_ZONE;
        rightOutakeMotor.setVelocity(TARGET_VELOCITY_FRONT_LAUNCH_ZONE);
        leftOutakeMotor.setVelocity(TARGET_VELOCITY_FRONT_LAUNCH_ZONE);
        pushServo.setPosition(SERVO_REST_POSITION);

        if ((leftOutakeMotor.getVelocity() >= setMinVelocity) && (rightOutakeMotor.getVelocity() >= setMinVelocity))
        {
            //Step 7. position servo into launch position
            pushServo.setPosition(SERVO_LAUNCH_POSITION);
            sleep(SLEEP_BEFORE_RESET_SERVO_POSITION);
            pushServo.setPosition(SERVO_REST_POSITION);


        }

    }

    public void stopOutake()
    {
        rightOutakeMotor.setVelocity(STOP_VELOCITY);
        leftOutakeMotor.setVelocity(STOP_VELOCITY);
        pushServo.setPosition(SERVO_REST_POSITION);


    }




}
