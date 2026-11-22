package org.firstinspires.ftc.teamcode.opmodes.Autonomous;

import static org.firstinspires.ftc.teamcode.opmodes.Autonomous.testMatch.AIM_TIMEOUT_MS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commands.IntakeFlower.IntakeFlowerDown;
import org.firstinspires.ftc.teamcode.commands.IntakeFlower.IntakeFlowerUp;
import org.firstinspires.ftc.teamcode.commands.TransferSequence;
import org.firstinspires.ftc.teamcode.commands.intake.intakeOff;
import org.firstinspires.ftc.teamcode.commands.intake.intakeOn;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.gate.Gate;
import org.firstinspires.ftc.teamcode.subsystems.indicator.Indicator;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intakeFlower.IntakeFlower;
import org.firstinspires.ftc.teamcode.subsystems.pollensensor.PollenSensor;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretHistory;
import org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMotor;
import org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMotorConstants;
import org.firstinspires.ftc.teamcode.util.PosePersistency;
import org.firstinspires.ftc.teamcode.vision.Limelight;

@Autonomous(name = "farFlowerAuto", group = "Autonomous")
public class farFlowerAuto extends CommandOpMode {

    private static final long DELAY_STEP_MS = 500;
    private static final long MAX_DELAY_MS = 25000;

    private Follower follower;
    private Intake intake;
    private IntakeFlower flower;
    private Gate gate;
    private PollenSensor pollenSensor;
    private Indicator indicator;
    private Shooter shooter;
    private TurretMotor turret;
    private Limelight limelight;
    private TelemetryData telemetryData;
    private Field.Alliance alliance = Field.Alliance.BLUE;

    private long shootDelayMs = 0;
    private boolean lastUp = false;
    private boolean lastDown = false;

    private Pose start;
    private Pose startToFlower;
    private Pose startToFlowerControl1;

    private void buildPoses() {
        PoseFactory poseFactory = Field.poses(alliance);
        start = poseFactory.of(79.3306, 8.3926, 90);
        startToFlower = poseFactory.of(132.1767, 27.5797, 320);
        startToFlowerControl1 = poseFactory.of(92.5399, 38.9554, 0);
    }

    public Command autoRoutine() {
        return new SequentialCommandGroup(
                new WaitCommand(shootDelayMs),
                new WaitUntilCommand(() -> turret.isAimed() && shooter.flywheelReady()).withTimeout(AIM_TIMEOUT_MS),
                new TransferSequence(intake, gate),
                new FollowPathCommand(follower, startToFlower())
        );
    }

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        TurretHistory history = new TurretHistory();
        limelight = new Limelight(hardwareMap, history,
                () -> TurretMotorConstants.MOTOR_TURRET_FWD, () -> TurretMotorConstants.MOTOR_TURRET_LEFT);
        turret = new TurretMotor(hardwareMap, limelight, history, follower::pose);
        shooter = new Shooter(hardwareMap, turret::getDistance);
        intake = new Intake(hardwareMap);
        flower = new IntakeFlower(hardwareMap, intake);
        gate = new Gate(hardwareMap);
        pollenSensor = new PollenSensor(hardwareMap);
        indicator = new Indicator(hardwareMap, turret::isAimed, pollenSensor::isPresent);
        register(limelight, turret, shooter, intake, flower, gate, pollenSensor, indicator);
        flower.setState(IntakeFlower.State.UP);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryData = new TelemetryData(telemetry);

        alliance = PosePersistency.getAlliance(Field.Alliance.BLUE);
    }

    @Override
    public void initialize_loop() {
        if (gamepad1.x) alliance = Field.Alliance.BLUE;
        if (gamepad1.b) alliance = Field.Alliance.RED;
        if (gamepad1.y) turret.zeroHere();

        if (gamepad1.dpad_up && !lastUp) shootDelayMs = Math.min(shootDelayMs + DELAY_STEP_MS, MAX_DELAY_MS);
        if (gamepad1.dpad_down && !lastDown) shootDelayMs = Math.max(shootDelayMs - DELAY_STEP_MS, 0);
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        buildPoses();
        turret.readEncoder();
        telemetryData.addData("Controls", "X = BLUE, B = RED, Y = zero turret here (point it forward), DPAD U/D = shoot delay +/- " + DELAY_STEP_MS + " ms");
        telemetryData.addData("Alliance", alliance);
        telemetryData.addData("Start pose", String.format("x=%.1f y=%.1f h=%.0f deg", start.x(), start.y(), Math.toDegrees(start.heading())));
        telemetryData.addData("First shot delay s", shootDelayMs / 1000.0);
        telemetryData.addData("Turret deg", Math.toDegrees(turret.getAngle()));
        telemetryData.addData("Encoder fault", turret.isEncoderFault());
        telemetryData.update();
    }

    @Override
    public void preRun() {
        buildPoses();
        follower.setPose(start);
        follower.update();
        PosePersistency.saveAlliance(alliance);
        PosePersistency.savePose(start);
        turret.setAlliance(alliance);
        turret.setState(TurretMotor.State.AIM);
        shooter.setState(Shooter.State.DISTANCE);
        schedule(autoRoutine());
    }

    @Override
    public void run() {
        super.run();
        follower.update();
        PosePersistency.savePose(follower.pose());

        telemetryData.addData("Alliance", alliance);
        telemetryData.addData("First shot delay s", shootDelayMs / 1000.0);
        telemetryData.addData("Pose", follower.pose());
        telemetryData.addData("Intake state", intake.getState());
        telemetryData.addData("Flower state", flower.getState());
        if (follower.currentPath() != null) {
            telemetryData.addData("Current path distance remaining", follower.distanceToEndpoint());
        }
        shooter.addTelemetry(telemetryData);
        turret.addTelemetry(telemetryData);
        telemetryData.update();
    }

    @Override
    public void end() {
        PosePersistency.savePose(follower.pose());
        intake.setState(Intake.State.OFF);
        shooter.setState(Shooter.State.OFF);
    }

    public Path startToFlower() {
        return Paths.curve(start, startToFlowerControl1, startToFlower).linear(start, startToFlower);
    }
}
