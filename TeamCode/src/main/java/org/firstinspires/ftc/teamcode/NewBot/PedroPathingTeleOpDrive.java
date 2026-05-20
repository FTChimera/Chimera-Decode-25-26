package org.firstinspires.ftc.teamcode.NewBot;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Supplier;

@SuppressWarnings("SpellCheckingInspection")
@Configurable
@TeleOp
public class PedroPathingTeleOpDrive extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    boolean isRobotCentric = false;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    public static int DegreesOffset = 90;

    @Override
    public void init() {
        startingPose = MatchState.getStartingPose();
        follower = Constants.createPedroFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? Constants.CHIMERA_TESTING_POSE: startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, Constants.RED_PARKING)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 0.8))
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        if (!automatedDrive) {
            //Make the last parameter false for field-centric

            //This is the normal version to use in the TeleOp
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                     isRobotCentric,
                    isRobotCentric ? 0 : Math.toRadians(DegreesOffset)
             );

        }
        if (gamepad1.backWasPressed()) {
            isRobotCentric = !isRobotCentric;
        }
        if (gamepad1.dpadDownWasPressed()) {
            follower.setPose(follower.getPose().setHeading(0));
        }

        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (gamepad1.bWasPressed()) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // Call this once per loop after any teleop/path command updates.
        follower.update();
        telemetryM.update();

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.debug("Drive Mode", isRobotCentric?"Robot Centric":"Field Centric");
    }
}
