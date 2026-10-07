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

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        turret = new Turret(hardwareMap, new TurretHistory());
        register(turret);

        gamepadEx1 = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryData = new TelemetryData(telemetry);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> targetDeg += STEP_DEG);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> targetDeg -= STEP_DEG);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> targetDeg = 0);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            fieldLock = !fieldLock;

            fieldAngle = Math.toRadians(targetDeg) + follower.pose().heading();
        });
    }

    @Override
    public void initialize_loop() {
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
        turret.setManualAngle(Math.toRadians(targetDeg));

        super.run();

        telemetryData.addData("Controls", "dpad L/R = +-15 deg, A = 0, B = field lock, hand-turn only during init");
        telemetryData.addData("Field lock", fieldLock);
        telemetryData.addData("Raw deg", turret.getRawDeg());
        telemetryData.addData("Angle deg", Math.toDegrees(turret.getAngle()));
        telemetryData.addData("Target deg", Math.toDegrees(turret.getSetpoint()));
        telemetryData.addData("Error deg", Math.toDegrees(turret.getSetpoint() - turret.getAngle()));
        telemetryData.addData("Velocity deg/s", Math.toDegrees(turret.getVelocity()));
        telemetryData.addData("Power", turret.getPower());
        telemetryData.addData("In range", turret.isInRange());
        telemetryData.addData("Settled", turret.isSettled());
        telemetryData.addData("Encoder fault", turret.isEncoderFault());
        telemetryData.update();
    }
}
