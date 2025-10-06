package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.PinpointView;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Objects;



@Config
public final class PinpointLocalizer implements Localizer {
    public static class Params {
        public double parYTicks = 1623.1111622917413; // y position of the parallel encoder (in tick units)
        public double perpXTicks = 2259.617906730483; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver driver;
    public GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    // inPerTick = 0.00198429103;
    // mmPerTick = 0.050400992162;
    // X_pod_offset = mmPerTick *  PARAMS.parYTicks = 81.806412968720763
    // Y_pod_offset = mmPerTick * PARAMS.perpXTicks = 113.886942098352163
    // parYTicks = 1623.1111622917413; // y position of the parallel encoder (in tick units)
    // perpXTicks = 2259.617906730483;

    public PinpointLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        // TODO-Completed: make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        double mmPerTick = inPerTick * 25.4;

        // TODO-Changed: Original Code was as follows
        //driver.setEncoderResolution(1 / mmPerTick);
        //  driver.setOffsets(mmPerTick * PARAMS.parYTicks, mmPerTick * PARAMS.perpXTicks);
        // TODO-Changed: New Incoming change from Fork Code was as follows
        //double mmPerTick = 25.4 * inPerTick;
        //driver.setOffsets(mmPerTick * PARAMS.parYTicks, mmPerTick * PARAMS.perpXTicks, DistanceUnit.MM);
        // TODO-Changed: Accepted Incoming change from Fork Code as follows
        driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);

        // TODO-New Values: Measured from the Center of the Robot
        // X_pod_offset = 0 mm from the center of the Robot
        // Y_pod_offset = 6 inches = 152.4 mm from the center of the Robot.
        driver.setOffsets(0, 152.4, DistanceUnit.MM);

        // TODO-Changed: reverse encoder directions if needed
        // Parallel Dead Wheels(should increase forward)
        // Perpendicular Dead Wheels(should increase leftward)

        initialParDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();

        txWorldPinpoint = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }

    @Override
    public PoseVelocity2d update() {
        driver.update();
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            txPinpointRobot = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), driver.getHeading(UnnormalizedAngleUnit.RADIANS));
            Vector2d worldVelocity = new Vector2d(driver.getVelX(DistanceUnit.INCH), driver.getVelY(DistanceUnit.INCH));
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);

            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }
}
