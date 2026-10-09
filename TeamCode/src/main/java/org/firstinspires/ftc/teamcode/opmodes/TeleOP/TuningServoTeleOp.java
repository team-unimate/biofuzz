package org.firstinspires.ftc.teamcode.opmodes.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.turret.ShotTable;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretHistory;
import org.firstinspires.ftc.teamcode.subsystems.turretservo.TurretServo;
import org.firstinspires.ftc.teamcode.vision.CellLocator;
import org.firstinspires.ftc.teamcode.vision.CellObservation;
import org.firstinspires.ftc.teamcode.vision.Limelight;

import java.util.Locale;

@Config
@TeleOp(name = "Tuning Servo TeleOp", group = "Tuning")
public class TuningServoTeleOp extends CommandOpMode {
    public static double START_X_IN = 72;
    public static double START_Y_IN = 72;
    public static double START_HEADING_DEG = 90;
    public static double MERGE_DISTANCE_IN = 3;

    private Follower follower;
    private TurretServo turret;
    private Limelight limelight;
    private TelemetryData telemetryData;
    private ShotTable shotTable;

    private Field.Alliance alliance = Field.Alliance.BLUE;
    private boolean shooterOn = true;

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        TurretHistory history = new TurretHistory();
        limelight = new Limelight(hardwareMap, history);
        turret = new TurretServo(hardwareMap, limelight, history);
        register(limelight, turret);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryData = new TelemetryData(telemetry);
        shotTable = new ShotTable(MERGE_DISTANCE_IN);

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                () -> TurretConstants.TUNING_SHOOTER = !TurretConstants.TUNING_SHOOTER);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> shooterOn = !shooterOn);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
            if (TurretConstants.TUNING_SHOOTER) shotTable.record(turret.getDistance(), TurretConstants.TUNING_VELOCITY);
        });
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> shotTable.clear());
    }

    @Override
    public void initialize_loop() {
        if (gamepad1.x) alliance = Field.Alliance.BLUE;
        if (gamepad1.b) alliance = Field.Alliance.RED;

        telemetryData.addData("Controls", "X = BLUE, B = RED");
        telemetryData.addData("Alliance", alliance);
        telemetryData.update();
    }

    @Override
    public void preRun() {
        follower.setPose(new Pose(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG)));
        turret.setAlliance(alliance);
    }

    @Override
    public void run() {
        follower.manual(ManualDrive.fieldCentric(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                follower.pose().heading()
        ));
        follower.update();
        turret.updateBotPose(follower.pose());
        turret.setShooterEnabled(shooterOn);
        if (TurretConstants.USE_MANUAL_ANGLE) {
            turret.setManualAngle(Math.toRadians(TurretConstants.MANUAL_ANGLE_DEG));
        } else {
            turret.clearManualAngle();
        }

        super.run();

        telemetryData.addData("Controls", "A = shooter AUTO/TUNING, B = shooter OFF/ON, dpad up = record, dpad down = clear");
        telemetryData.addData("Shooter mode", !shooterOn ? "OFF" : TurretConstants.TUNING_SHOOTER ? "TUNING" : "AUTO");
        telemetryData.addData("Manual angle", TurretConstants.USE_MANUAL_ANGLE
                ? TurretConstants.MANUAL_ANGLE_DEG + " deg" : "off (auto aim)");
        telemetryData.addData("Pose", String.format(Locale.US, "%.1f, %.1f, %.1f deg",
                follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading())));
        turret.addTelemetry(telemetryData);

        StringBuilder ids = new StringBuilder();
        for (CellLocator.TagPoint tag : limelight.getTagPoints()) ids.append(tag.id).append(' ');
        telemetryData.addData("Seen tag IDs", ids.toString().trim());
        telemetryData.addData("Frame age ms", limelight.getFrameAgeMs());
        CellObservation obs = turret.getCell() == null ? null : limelight.getObservation(turret.getCell());
        telemetryData.addData("Cell tags / bearing deg / distance in", obs == null ? "not seen"
                : String.format(Locale.US, "%d / %.1f / %.1f", obs.tagCount(), Math.toDegrees(obs.bearing), obs.distance));

        telemetryData.addData("Recorded points", shotTable.size());
        telemetryData.addData("Paste dist", shotTable.distanceLine());
        telemetryData.addData("Paste vel", shotTable.velocityLine());

        if (TurretConstants.SHOW_TAGS) {
            for (CellLocator.TagPoint tag : limelight.getTagPoints()) {
                telemetryData.addData("Tag " + tag.id + " fwd/left/up in",
                        String.format(Locale.US, "%.1f / %.1f / %.1f", tag.forward, tag.left, tag.up));
            }
        }
        telemetryData.update();
    }

    @Override
    public void end() {
        TurretConstants.TUNING_SHOOTER = false;
        TurretConstants.USE_MANUAL_ANGLE = false;
    }
}
