package org.firstinspires.ftc.teamcode.opmodes.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretHistory;
import org.firstinspires.ftc.teamcode.vision.CellLocator;
import org.firstinspires.ftc.teamcode.vision.CellObservation;
import org.firstinspires.ftc.teamcode.vision.Limelight;

@TeleOp(name = "Vision Test", group = "Tuning")
public class VisionTestTeleOp extends CommandOpMode {
    private static final double STEP_DEG = 15;

    private Follower follower;
    private Turret turret;
    private Limelight limelight;
    private GamepadEx gamepadEx1;
    private TelemetryData telemetryData;

    private double targetDeg = 0;

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        TurretHistory history = new TurretHistory();
        limelight = new Limelight(hardwareMap, history);
        turret = new Turret(hardwareMap, history);

        register(limelight, turret);

        gamepadEx1 = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryData = new TelemetryData(telemetry);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> targetDeg += STEP_DEG);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> targetDeg -= STEP_DEG);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> targetDeg = 0);
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
        turret.setManualAngle(Math.toRadians(targetDeg));

        super.run();

        telemetryData.addData("Controls", "dpad L/R = turret +-15 deg, A = 0");
        telemetryData.addData("Limelight connected", limelight.isConnected());
        telemetryData.addData("Frame age ms", limelight.getFrameAgeMs());
        telemetryData.addData("Turret deg", Math.toDegrees(turret.getAngle()));

        StringBuilder ids = new StringBuilder();
        for (CellLocator.TagPoint tag : limelight.getTagPoints()) {
            ids.append(tag.id).append(' ');
            telemetryData.addData("Tag " + tag.id + " fwd/left/up in",
                    String.format("%.1f / %.1f / %.1f", tag.forward, tag.left, tag.up));
        }
        telemetryData.addData("Seen tag IDs", ids.toString().trim());

        for (CellObservation obs : limelight.getObservations().values()) {
            telemetryData.addData(obs.cell + " tags", obs.tagCount());
            telemetryData.addData(obs.cell + " bearing deg", Math.toDegrees(obs.bearing));
            telemetryData.addData(obs.cell + " distance in", obs.distance);
            telemetryData.addData(obs.cell + " row yaw deg", Math.toDegrees(obs.rowYaw));
            telemetryData.addData(obs.cell + " field x/y", String.format("%.1f / %.1f", obs.fieldX, obs.fieldY));
        }
        telemetryData.update();
    }
}
