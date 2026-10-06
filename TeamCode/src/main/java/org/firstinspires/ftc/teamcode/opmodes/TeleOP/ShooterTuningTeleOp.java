package org.firstinspires.ftc.teamcode.opmodes.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;

/**
 * Runs just the shooter flywheel so its RPM presets and PIDF gains (ShooterConstants) can be
 * tuned live from the FTC Dashboard while watching the actual vs. target RPM telemetry.
 */
@TeleOp(name = "Shooter Tuning", group = "Tuning")
public class ShooterTuningTeleOp extends CommandOpMode {
    private Shooter shooter;
    private GamepadEx gamepadEx1;
    private TelemetryData telemetryData;

    @Override
    public void initialize() {
        super.reset();

        shooter = new Shooter(hardwareMap);
        register(shooter);

        gamepadEx1 = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryData = new TelemetryData(telemetry);
    }

    @Override
    public void run() {
        super.run();

        if (gamepadEx1.getButton(GamepadKeys.Button.A)) {
            shooter.setTargetRPM(ShooterConstants.HIVE_RPM);
        } else if (gamepadEx1.getButton(GamepadKeys.Button.X)) {
            shooter.stop();
        }

        telemetryData.addData("Controls", "A = HIVE_RPM, X = stop. Tune kP/kI/kD/kF and the RPM preset from the dashboard.");
        telemetryData.addData("Target RPM", shooter.getTargetRPM());
        telemetryData.addData("Current RPM", shooter.getRPM());
        telemetryData.addData("Error RPM", shooter.getTargetRPM() - shooter.getRPM());
        telemetryData.addData("Power", shooter.getPower());
        telemetryData.addData("At Target", shooter.atTarget());
        telemetryData.addData("kP", format(ShooterConstants.kP));
        telemetryData.addData("kI", format(ShooterConstants.kI));
        telemetryData.addData("kD", format(ShooterConstants.kD));
        telemetryData.addData("kF", format(ShooterConstants.kF));
        telemetryData.update();
    }

    /** Fixed-point formatting for telemetry - scientific notation (e.g. "1.0E-6") is hard to read at a glance. */
    private static String format(double value) {
        return String.format("%.8f", value);
    }
}
