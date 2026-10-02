package org.firstinspires.ftc.teamcode.opmodes.TeleOP;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commands.field.TrackCellCommand;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Track Cell Test")
public class TrackCellTeleOp extends CommandOpMode {
    private static final Pose START_POSE = new Pose(133, 133.8594,  Math.toRadians(-90));

    private Follower follower;
    private GamepadEx gamepadEx1;
    private TelemetryData telemetryData;
    private Field.Alliance alliance = Field.Alliance.BLUE;
    private TrackCellCommand trackCellCommand;

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(START_POSE);
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

        telemetryData.addData("Controls", "X = BLUE alliance, B = RED alliance");
        telemetryData.addData("Alliance", alliance);
        telemetryData.update();
    }

    @Override
    public void preRun() {
        // The alliance is fixed once the chooser closes (start is pressed), so the command can be
        // built here with whatever was picked during initialize_loop.
        trackCellCommand = new TrackCellCommand(follower, alliance);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whileHeld(trackCellCommand);
    }

    @Override
    public void run() {
        super.run();

        boolean tracking = gamepadEx1.getButton(GamepadKeys.Button.A);
        if (!tracking) {
            // Pedro uses +x forwards, +y left and counterclockwise positive, while the gamepad sticks read
            // positive to the right and down, so all three axes are negated (same as Pedro's own quickstart).
            follower.manual(ManualDrive.fieldCentric(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    follower.pose().heading()
            ));
        }
        follower.update();

        telemetryData.addData("X", follower.pose().x());
        telemetryData.addData("Y", follower.pose().y());
        telemetryData.addData("Heading", follower.pose().heading());
        telemetryData.addData("Alliance", alliance);
        telemetryData.addData("Tracking", tracking ? Field.getCell(alliance, follower.pose()) : "manual");
        telemetryData.update();
    }
}
