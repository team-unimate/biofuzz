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
import org.firstinspires.ftc.teamcode.field.HiveCells;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.turret.ShotTable;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretHistory;
import org.firstinspires.ftc.teamcode.util.PosePersistency;
import org.firstinspires.ftc.teamcode.vision.Limelight;

@Config
@TeleOp(name = "Shooter LUT Tuning", group = "Tuning")
public class ShooterLutTuningTeleOp extends CommandOpMode {
    public static double START_X_IN = 72;
    public static double START_Y_IN = 72;
    public static double START_HEADING_DEG = 90;
    public static double COARSE_STEP = 50;
    public static double FINE_STEP = 10;
    public static double START_VELOCITY = 1200;
    public static double MERGE_DISTANCE_IN = 3;

    private Follower follower;
    private Turret turret;
    private GamepadEx gamepadEx1;
    private TelemetryData telemetryData;
    private ShotTable shotTable;

    private Field.Alliance alliance;
    private HiveCells.Cell startCell;
    private boolean useSavedPose;
    private double velocity;

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        TurretHistory history = new TurretHistory();
        Limelight limelight = new Limelight(hardwareMap, history);
        turret = new Turret(hardwareMap, limelight, history);
        register(limelight, turret);

        gamepadEx1 = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryData = new TelemetryData(telemetry);
        shotTable = new ShotTable(MERGE_DISTANCE_IN);

        alliance = PosePersistency.getAlliance(Field.Alliance.BLUE);
        startCell = PosePersistency.getUpCell();
        useSavedPose = PosePersistency.hasPose();
        velocity = START_VELOCITY;

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> velocity += COARSE_STEP);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> velocity = Math.max(0, velocity - COARSE_STEP));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> velocity += FINE_STEP);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> velocity = Math.max(0, velocity - FINE_STEP));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> shotTable.record(turret.getDistance(), velocity));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> shotTable.removeLast());
    }

    @Override
    public void initialize_loop() {
        if (gamepad1.x) alliance = Field.Alliance.BLUE;
        if (gamepad1.b) alliance = Field.Alliance.RED;
        if (gamepad1.dpad_up) startCell = alliance == Field.Alliance.RED ? HiveCells.Cell.RED_FAR : HiveCells.Cell.BLUE_FAR;
        if (gamepad1.dpad_down) startCell = alliance == Field.Alliance.RED ? HiveCells.Cell.RED_AUDIENCE : HiveCells.Cell.BLUE_AUDIENCE;
        if (gamepad1.y) useSavedPose = false;
        if (startCell != null && startCell.alliance != alliance) startCell = null;

        telemetryData.addData("Controls", "X = BLUE, B = RED, dpad up = FAR up, dpad down = AUDIENCE up, Y = ignore saved pose");
        telemetryData.addData("Alliance", alliance);
        telemetryData.addData("Start up cell", startCell == null ? "not set" : startCell);
        telemetryData.addData("Start pose", useSavedPose ? "saved " + PosePersistency.getPose(null) : "dashboard START_*");
        telemetryData.update();
    }

    @Override
    public void preRun() {
        Pose start = new Pose(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG));
        follower.setPose(useSavedPose ? PosePersistency.getPose(start) : start);
        turret.setAlliance(alliance);
        if (startCell != null) turret.setUpCell(startCell);
        turret.setAutoAim(true);
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
        turret.setShooterOverride(velocity);
        turret.setShooterEnabled(gamepad1.right_trigger > 0.5);

        super.run();

        telemetryData.addData("Controls", "RT = spin, dpad up/down = +-" + COARSE_STEP + ", dpad R/L = +-" + FINE_STEP
                + ", A = record, B = undo");
        telemetryData.addData("Distance in", turret.getDistance());
        telemetryData.addData("Status", turret.getStatus());
        telemetryData.addData("Vision", turret.getVisionState());
        telemetryData.addData("Target velocity", velocity);
        telemetryData.addData("Shooter velocity", turret.getShooterVelocity());
        telemetryData.addData("Recorded", shotTable.size());
        for (ShotTable.Shot shot : shotTable.sorted()) {
            telemetryData.addData(String.format("  %.1f in", shot.distance), shot.velocity);
        }
        telemetryData.addData("Paste dist", shotTable.distanceLine());
        telemetryData.addData("Paste vel", shotTable.velocityLine());
        telemetryData.update();
    }

    @Override
    public void end() {
        PosePersistency.savePose(follower.pose());
    }
}
