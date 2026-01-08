package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com. qualcomm.robotcore.hardware.DcMotor;

public class SlidesMovement {
    public DcMotor rightViperSlide;
    public DcMotor leftViperSlide;

    SlidesMovement(HardwareMap hardwareMap) {
        rightViperSlide = hardwareMap.dcMotor.get("rightViperSlide");
        leftViperSlide = hardwareMap.dcMotor.get("leftViperSlide");

        leftViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftViperSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightViperSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftViperSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightViperSlide.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public class slidesUp implements Action{
        @Override

        public boolean run(@NonNull TelemetryPacket packet){
            rightViperSlide.setTargetPosition(1400);
            rightViperSlide.setPower(1);
            leftViperSlide.setTargetPosition(1400);
            leftViperSlide.setPower(1);
            leftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            packet.put("Slides Up Right Viper slide Current Position", rightViperSlide.getCurrentPosition());
            packet.put("Slides Up Left Viper slide Current Position", leftViperSlide.getCurrentPosition());

            return false;
        }

    }

    public Action SlidesUp() {
        return new slidesUp();
}


    public class slidesDown implements Action {
        @Override

        public boolean run(@NonNull TelemetryPacket packet) {

            packet.put("Slides Down BEFORE Right Viper slide Current Position", rightViperSlide.getCurrentPosition());
            packet.put("Slides Down BEFORE Left Viper slide Current Position", leftViperSlide.getCurrentPosition());

            rightViperSlide.setTargetPosition(0);
            rightViperSlide.setPower(-1);
            leftViperSlide.setTargetPosition(0);
            leftViperSlide.setPower(-1);
            leftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


           // packet.addLine("Right Viper slide Current Position" + rightViperSlide.getCurrentPosition());
           // packet.addLine("Left Viper slide Current Position" + leftViperSlide.getCurrentPosition());
            packet.put("Slides Down AFTER Right Viper slide Current Position", rightViperSlide.getCurrentPosition());
            packet.put("Slides Down AFTER Left Viper slide Current Position", leftViperSlide.getCurrentPosition());


            if (rightViperSlide.getCurrentPosition() < 0) {

                leftViperSlide.setTargetPosition(0);
                rightViperSlide.setTargetPosition(0);

            }



            return false;
        }

    }

    public Action SlidesDown() {

        return new slidesDown();
    }




}
