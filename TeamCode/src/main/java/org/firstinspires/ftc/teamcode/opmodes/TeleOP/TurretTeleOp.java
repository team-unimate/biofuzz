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

import org.firstinspires.ftc.teamcode.commands.ShootWhenReady;
import org.firstinspires.ftc.teamcode.commands.turret.SetTurretSide;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.HiveCells;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretHistory;
import org.firstinspires.ftc.teamcode.util.PosePersistency;
import org.firstinspires.ftc.teamcode.vision.CellLocator;
import org.firstinspires.ftc.teamcode.vision.CellObservation;
import org.firstinspires.ftc.teamcode.vision.Limelight;

@Config
@TeleOp(name = "Turret TeleOp")
public class TurretTeleOp extends CommandOpMode {
    public static double START_X_IN = 72;
    public static double START_Y_IN = 72;
    public static double START_HEADING_DEG = 90;

    private Follower follower;
    private Turret turret;
    private Limelight limelight;
    private GamepadEx driver;
    private GamepadEx operator;
    private TelemetryData telemetryData;

    private Field.Alliance alliance;
    private HiveCells.Cell startCell;
    private boolean useSavedPose;
    private boolean shooterOn = false;

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        TurretHistory history = new TurretHistory();
        limelight = new Limelight(hardwareMap, history);
        turret = new Turret(hardwareMap, limelight, history);
        register(limelight, turret);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryData = new TelemetryData(telemetry);

        alliance = PosePersistency.getAlliance(Field.Alliance.BLUE);
        startCell = PosePersistency.getUpCell();
        useSavedPose = PosePersistency.hasPose();

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> {
            shooterOn = !shooterOn;
            turret.setShooterEnabled(shooterOn);
        });
        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ShootWhenReady(turret));
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> turret.setUpCell(farCell(alliance)));
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> turret.setUpCell(audienceCell(alliance)));
    }

    @Override
    public void initialize_loop() {
        if (gamepad1.x) alliance = Field.Alliance.BLUE;
        if (gamepad1.b) alliance = Field.Alliance.RED;
        if (gamepad1.dpad_up) startCell = farCell(alliance);
        if (gamepad1.dpad_down) startCell = audienceCell(alliance);
        if (gamepad1.y) useSavedPose = false;
        if (startCell != null && startCell.alliance != alliance) startCell = null;

        turret.readEncoder();
        telemetryData.addData("Controls", "X = BLUE, B = RED, dpad up = FAR up, dpad down = AUDIENCE up, Y = ignore saved pose");
        telemetryData.addData("Alliance", alliance);
        telemetryData.addData("Start up cell", startCell == null ? "not set" : startCell);
        telemetryData.addData("Start pose", useSavedPose ? "saved " + PosePersistency.getPose(null) : "dashboard START_*");
        telemetryData.addData("Turret deg", Math.toDegrees(turret.getAngle()));
        telemetryData.addData("Encoder fault", turret.isEncoderFault());
        telemetryData.update();
    }

    @Override
    public void preRun() {
        Pose start = new Pose(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG));
        follower.setPose(useSavedPose ? PosePersistency.getPose(start) : start);
        schedule(new SetTurretSide(turret, alliance));
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

        super.run();

        PosePersistency.savePose(follower.pose());
        PosePersistency.saveUpCell(turret.getUpCell());

        telemetryData.addData("Status", turret.getStatus());
        HiveCells.Cell cell = turret.getUpCell();
        telemetryData.addData("Target cell", cell == null ? "none" : cell + (turret.isUpCellConfirmed() ? "" : " (guess)"));
        telemetryData.addData("Distance in", turret.getDistance());
        telemetryData.addData("Vision", turret.getVisionState());
        CellObservation obs = turret.getLastObservation();
        telemetryData.addData("Vision tags", obs == null ? 0 : obs.tagCount());

        StringBuilder ids = new StringBuilder();
        for (CellLocator.TagPoint tag : limelight.getTagPoints()) ids.append(tag.id).append(' ');
        telemetryData.addData("Seen tag IDs", ids.toString().trim());

        telemetryData.addData("odomAngle", Math.toDegrees(turret.getOdomAngle()));
        telemetryData.addData("bias", Math.toDegrees(turret.getBias()));
        telemetryData.addData("turretTarget", Math.toDegrees(turret.getCommandedTarget()));
        telemetryData.addData("turretAngle", Math.toDegrees(turret.getAngle()));
        telemetryData.addData("Shooter", shooterOn ? "on" : "off");
        telemetryData.addData("Shooter target", turret.getShooterTarget());
        telemetryData.addData("Shooter velocity", turret.getShooterVelocity());
        telemetryData.addData("Encoder fault", turret.isEncoderFault());
        telemetryData.update();
    }

    @Override
    public void end() {
        PosePersistency.savePose(follower.pose());
        PosePersistency.saveUpCell(turret.getUpCell());
    }

    private static HiveCells.Cell farCell(Field.Alliance alliance) {
        return alliance == Field.Alliance.RED ? HiveCells.Cell.RED_FAR : HiveCells.Cell.BLUE_FAR;
    }

    private static HiveCells.Cell audienceCell(Field.Alliance alliance) {
        return alliance == Field.Alliance.RED ? HiveCells.Cell.RED_AUDIENCE : HiveCells.Cell.BLUE_AUDIENCE;
    }
}
