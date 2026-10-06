package org.firstinspires.ftc.teamcode.opmodes.TeleOP;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.vision.AllianceTags;
import org.firstinspires.ftc.teamcode.vision.AprilTagAimController;
import org.firstinspires.ftc.teamcode.vision.AprilTagField;
import org.firstinspires.ftc.teamcode.vision.AprilTagTarget;
import org.firstinspires.ftc.teamcode.vision.Limelight;

/**
 * Field-centric Pedro Pathing drive that, while RIGHT_BUMPER is held, replaces the rotational
 * stick input with a heading correction aimed at the current alliance's AprilTag - translation
 * stays fully driver-controlled and Pedro's Follower is never bypassed, only fed a different turn
 * power for that loop.
 * <p>
 * The alliance chooser (X = BLUE, B = RED) during init matches {@link TrackCellTeleOp}'s
 * convention; it determines the set of tag IDs ({@link AllianceTags}) considered valid to aim at -
 * whichever one of them is currently visible is tracked, and if none are visible the last one
 * tracked is held briefly (see {@link AprilTagAimController}).
 */
@TeleOp(name = "AprilTag Aim Test", group = "Vision")
public class AprilTagAimTeleOp extends CommandOpMode {
    private static final Pose START_POSE = new Pose(133, 133.8594, Math.toRadians(-90));

    private Follower follower;
    private Limelight limelight;
    private GamepadEx gamepadEx1;
    private TelemetryData telemetryData;

    private final AprilTagAimController aimController = new AprilTagAimController();
    private Field.Alliance alliance = Field.Alliance.BLUE;
    private int trackedTagId = -1;

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(START_POSE);

        limelight = new Limelight(hardwareMap);
        register(limelight);

        gamepadEx1 = new GamepadEx(gamepad1);
        telemetryData = new TelemetryData(telemetry);
    }

    @Override
    public void initialize_loop() {
        if (gamepad1.x) {
            alliance = Field.Alliance.BLUE;
        } else if (gamepad1.b) {
            alliance = Field.Alliance.RED;
        }

        telemetryData.addData("Controls", "X = BLUE alliance, B = RED alliance, hold RIGHT_BUMPER to aim");
        telemetryData.addData("Alliance", alliance);
        telemetryData.update();
    }

    @Override
    public void run() {
        super.run();

        Pose pose = follower.pose();

        // Pedro uses +x forwards, +y left and counterclockwise positive, while the gamepad sticks read
        // positive to the right and down, so all three axes are negated (same as Pedro's own quickstart).
        DrivePowers sticks = ManualDrive.fieldCentric(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                pose.heading()
        );

        int[] candidateTagIds = AllianceTags.tagIdsFor(alliance);

        boolean aiming = gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        boolean tagVisible = false;
        double turnPower = sticks.turn();

        if (aiming) {
            LLResultTypes.FiducialResult visible = limelight.getFirstVisible(candidateTagIds);
            tagVisible = visible != null;
            if (tagVisible) {
                trackedTagId = visible.getFiducialId();
            }
            turnPower = trackedTagId >= 0
                    ? aimController.aimAtAprilTag(pose.x(), pose.y(), pose.heading(), trackedTagId, tagVisible)
                    : 0;
        } else {
            aimController.reset();
            trackedTagId = -1;
        }

        follower.manual(new DrivePowers(sticks.forward(), sticks.strafe(), turnPower));
        follower.update();

        AprilTagTarget target = AprilTagField.get(trackedTagId);
        double targetHeading = aimController.getLastTargetHeading();

        telemetryData.addData("Alliance", alliance);
        telemetryData.addData("Target Tag ID", trackedTagId);
        telemetryData.addData("Tag Visible", tagVisible);
        telemetryData.addData("Robot X", pose.x());
        telemetryData.addData("Robot Y", pose.y());
        telemetryData.addData("Robot Heading (deg)", Math.toDegrees(pose.heading()));
        telemetryData.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
        telemetryData.addData("Heading Error (deg)",
                Math.toDegrees(AprilTagAimController.wrapHeadingError(targetHeading, pose.heading())));
        telemetryData.addData("Turn Power", turnPower);
        telemetryData.addData("Tag X", target != null ? target.x : Double.NaN);
        telemetryData.addData("Tag Y", target != null ? target.y : Double.NaN);
        telemetryData.update();
    }
}
