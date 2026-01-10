package org.firstinspires.ftc.teamcode2.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class LogitechSystem {
    long exposureTime = 10000; // in microseconds
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    public AprilTagPoseFtc AprilTagPose;
    public Pose3D pose;
    public int id;
    public LogitechSystem(HardwareMap hwMap) {
        tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(640,480))
                .build();
    }
    public void update() {
        AprilTagDetection tag = tagProcessor.getDetections().get(0);
        AprilTagPose = tag.ftcPose;
        id = tag.id;
        Position position = new Position();
        AngleUnit angleUnit = AngleUnit.RADIANS; // Find AngleUnit
        YawPitchRollAngles orientation = new YawPitchRollAngles(angleUnit, AprilTagPose.yaw, AprilTagPose.pitch, AprilTagPose.roll, exposureTime);
        position.x = tag.ftcPose.x;
        position.y = tag.ftcPose.y;
        position.z = tag.ftcPose.z;
        pose = new Pose3D(position,orientation);
    }
}
