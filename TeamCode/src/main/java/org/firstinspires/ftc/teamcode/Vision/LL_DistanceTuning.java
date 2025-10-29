package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
@TeleOp
public class LL_DistanceTuning extends OpMode {

    public Limelight3A limelight1;

    public static Integer ActDistance;

    public static boolean UpdateList;

    public static List<Integer> Distances;
    public static List<Double> Areas;


    @Override
    public void init(){
        limelight1 = hardwareMap.get(Limelight3A.class, "limelight");
        limelight1.pipelineSwitch(0); // Test Pipeline
    }

    @Override
    public void start() {limelight1.start();}

    @Override
    public void loop(){
        LLResult result = limelight1.getLatestResult();

        if (result!=null && result.isValid()) {
            Pose3D botpose = result.getBotpose_MT2();
            telemetry.addData("Target X", result.getTx());
            telemetry.addData("Target Area", result.getTa());
            telemetry.addData("Botpose", botpose.toString());
            telemetry.addData("Distance (actual)", ActDistance);
            telemetry.update();
            if (UpdateList) {
                Distances.add(ActDistance);
                Areas.add(result.getTa());
                UpdateList = false;
            }
        }



    }
}
