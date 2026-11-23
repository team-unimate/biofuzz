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

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMotor;
import org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMotorConstants;
import org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretHistory;
import org.firstinspires.ftc.teamcode.vision.CellLocator;
import org.firstinspires.ftc.teamcode.vision.CellObservation;
import org.firstinspires.ftc.teamcode.vision.Limelight;
import org.firstinspires.ftc.teamcode.vision.VisionTuning;

import java.util.Locale;

@Config
@TeleOp(name = "Vision Tuning", group = "Tuning")
public class VisionTuningTeleOp extends CommandOpMode {
    public static double START_X_IN = 133;
    public static double START_Y_IN = 132;
    public static double START_HEADING_DEG = -90;

    private Follower follower;
    private TurretMotor turret;
    private Limelight limelight;
    private TelemetryData telemetryData;


    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        TurretHistory history = new TurretHistory();
        limelight = new Limelight(hardwareMap, history);
        turret = new TurretMotor(hardwareMap, limelight, history, follower::pose);
        register(limelight, turret);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryData = new TelemetryData(telemetry);

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                () -> VisionTuning.AUTO_AIM = !VisionTuning.AUTO_AIM);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                () -> TurretMotorConstants.USE_VISION = !TurretMotorConstants.USE_VISION);
    }

    @Override
    public void initialize_loop() {

        turret.readEncoder();
        telemetryData.addData("Controls", "X = BLUE, B = RED. Point the turret forward, then press start");
        telemetryData.addData("Turret angle deg", Math.toDegrees(turret.getAngle()));
        telemetryData.update();
    }

    @Override
    public void preRun() {
        follower.setPose(new Pose(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG)));
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
        Pose pose = follower.pose();
        if (VisionTuning.AUTO_AIM) {
            turret.setState(TurretMotor.State.AIM);
        } else {
            turret.setManualAngle(Math.toRadians(VisionTuning.TURRET_DEG));
        }

        super.run();

        telemetryData.addData("Controls", "A = turret AUTO AIM / HOLD TURRET_DEG, B = vision correction ON/OFF");
        telemetryData.addData("Turret mode", VisionTuning.AUTO_AIM ? "AUTO AIM" : "HOLD " + VisionTuning.TURRET_DEG + " deg");
        telemetryData.addData("Turret angle deg", Math.toDegrees(turret.getAngle()));
        telemetryData.addData("Pose", String.format(Locale.US, "%.1f, %.1f, %.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        telemetryData.addData("Limelight connected", limelight.isConnected());
        telemetryData.addData("Frame age ms", limelight.getFrameAgeMs());

        for (CellLocator.TagPoint tag : limelight.getTagPoints()) {
            telemetryData.addData("Tag " + tag.id + " fwd / left / up in",
                    String.format(Locale.US, "%.1f / %.1f / %.1f", tag.forward, tag.left, tag.up));
        }

        if (turret.getCell() == null) {
            telemetryData.addData("Cell", "none");
            telemetryData.update();
            return;
        }

        double[] pivot = TurretMotor.pivot(pose.x(), pose.y(), pose.heading(),
                TurretMotorConstants.MOTOR_TURRET_FWD, TurretMotorConstants.MOTOR_TURRET_LEFT);
        double odomAngle = TurretMotor.odomAngle(pivot[0], pivot[1], pose.heading(),
                turret.getCell().pose.x(), turret.getCell().pose.y());
        double odomDistance = Math.hypot(turret.getCell().pose.x() - pivot[0], turret.getCell().pose.y() - pivot[1]);

        telemetryData.addData("Cell", turret.getCell());
        telemetryData.addData("Vision correction", TurretMotorConstants.USE_VISION ? "ON" : "OFF");
        telemetryData.addData("Tags seen (lock)", turret.isSeen());
        telemetryData.addData("Bias deg", Math.toDegrees(turret.getBias()));
        telemetryData.addData("Aim target deg (odom + bias)", Math.toDegrees(turret.getTarget()));
        telemetryData.addData("Aim distance in", turret.getDistance());
        telemetryData.addData("Aim error deg (target - turret)", Math.toDegrees(TurretMotor.wrap(turret.getTarget() - turret.getAngle())));
        telemetryData.addData("Aim offset deg", TurretMotorConstants.AIM_OFFSET_DEG);
        telemetryData.addData("Odom angle deg / distance in",
                String.format(Locale.US, "%.1f / %.1f", Math.toDegrees(odomAngle), odomDistance));

        CellObservation obs = limelight.getObservation(turret.getCell());
        if (obs == null) {
            telemetryData.addData("Vision", "cell not seen");
            telemetryData.update();
            return;
        }

        double visionAngle = TurretMotor.visionAngle(obs.turretAngleAtCapture, obs.bearing, pose.heading(), obs.headingAtCapture);
        telemetryData.addData("Vision tags", obs.tagCount());
        telemetryData.addData("Vision bearing deg (from turret)", Math.toDegrees(obs.bearing));
        telemetryData.addData("Vision angle deg / distance in",
                String.format(Locale.US, "%.1f / %.1f", Math.toDegrees(visionAngle), obs.distance));
        double disagree = Math.toDegrees(TurretMotor.wrap(visionAngle - odomAngle));
        telemetryData.addData("Vision - odom angle deg", disagree);
        telemetryData.addData("Vision accepted", Math.abs(disagree) <= TurretMotorConstants.VISION_MAX_DISAGREE_DEG);
        telemetryData.addData("Vision - odom distance in", obs.distance - odomDistance);
        if (VisionTuning.MEASURED_DISTANCE_IN > 0) {
            telemetryData.addData("Vision - tape distance in", obs.distance - VisionTuning.MEASURED_DISTANCE_IN);
        }
        telemetryData.addData("Row yaw deg", Math.toDegrees(obs.rowYaw));
        telemetryData.addData("Cell field x / y (Field.java)", String.format(Locale.US, "%.1f / %.1f",
                turret.getCell().pose.x(), turret.getCell().pose.y()));
        telemetryData.addData("Cell field x / y (vision)", String.format(Locale.US, "%.1f / %.1f",
                obs.fieldX, obs.fieldY));
        telemetryData.update();
    }

    @Override
    public void end() {
        VisionTuning.AUTO_AIM = false;
        TurretMotorConstants.USE_VISION = true;
    }
}
