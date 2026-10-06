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

/**
 * Turret acceptance tests 2 and 3 (TURRET.md): encoder reading and turret control.
 * <p>
 * Test 2: init with the turret anywhere and read "Angle deg" (absolute, no zeroing). Press X to
 * cut power and turn the turret by hand to +90 and -90.
 * Test 3: hold a fixed angle without oscillating, then press B (field lock) and spin the robot in
 * place with the right stick - the turret should keep pointing the same way on the field.
 * Tune TurretConstants live from the dashboard; graph "Target deg" against "Angle deg".
 */
@TeleOp(name = "Turret Tuning", group = "Tuning")
public class TurretTuningTeleOp extends CommandOpMode {
    private static final double STEP_DEG = 15;

    private Follower follower;
    private Turret turret;
    private GamepadEx gamepadEx1;
    private TelemetryData telemetryData;

    private double targetDeg = 0;
    private boolean fieldLock = false;
    private double fieldAngle = 0;
    private boolean limp = false;

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        turret = new Turret(hardwareMap);
        register(turret);

        gamepadEx1 = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryData = new TelemetryData(telemetry);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> targetDeg += STEP_DEG);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> targetDeg -= STEP_DEG);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> targetDeg = 0);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            fieldLock = !fieldLock;
            // Lock onto the field direction the turret is aimed at right now.
            fieldAngle = Math.toRadians(targetDeg) + follower.pose().heading();
        });
        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            limp = !limp;
            turret.setLimp(limp);
        });
    }

    @Override
    public void initialize_loop() {
        // Test 2: the angle must already be right before start. Read only, no motion in init.
        turret.readEncoder();
        telemetryData.addData("Raw deg", turret.getRawDeg());
        telemetryData.addData("Angle deg", Math.toDegrees(turret.getAngle()));
        telemetryData.addData("Encoder fault", turret.isEncoderFault());
        telemetryData.update();
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

        if (fieldLock) {
            targetDeg = Math.toDegrees(fieldAngle - follower.pose().heading());
        }
        turret.setTargetAngle(Math.toRadians(targetDeg));

        super.run();

        telemetryData.addData("Controls", "dpad L/R = +-15 deg, A = 0, B = field lock, X = limp (hand-turn)");
        telemetryData.addData("Field lock", fieldLock);
        telemetryData.addData("Limp", limp);
        telemetryData.addData("Raw deg", turret.getRawDeg());
        telemetryData.addData("Angle deg", Math.toDegrees(turret.getAngle()));
        telemetryData.addData("Target deg", Math.toDegrees(turret.getCommandedTarget()));
        telemetryData.addData("Error deg", Math.toDegrees(turret.getCommandedTarget() - turret.getAngle()));
        telemetryData.addData("Velocity deg/s", Math.toDegrees(turret.getVelocity()));
        telemetryData.addData("Power", turret.getPower());
        telemetryData.addData("Reachable", turret.isReachable());
        telemetryData.addData("Settled", turret.isSettled());
        telemetryData.addData("Encoder fault", turret.isEncoderFault());
        telemetryData.addData("Robot turn rate deg/s", Math.toDegrees(turret.getRobotTurnRate()));
        telemetryData.update();
    }
}
