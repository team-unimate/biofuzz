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
import com.seattlesolvers.solverslib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretHistory;
import org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMotor;
import org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMotorConstants;
import org.firstinspires.ftc.teamcode.util.PosePersistency;
import org.firstinspires.ftc.teamcode.vision.Limelight;

@Autonomous(name = "flowerAuto", group = "Autonomous")
public class flowerAuto extends CommandOpMode {

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
    private Field.Alliance alliance;

    private Pose start;
    private Pose startToFlower;
    private Pose flowerToShoot;

    private void buildPoses(Field.Alliance alliance) {
        PoseFactory poseFactory = Field.poses(alliance);
        start = poseFactory.of(133.0491, 133.3243, -90);
        startToFlower = poseFactory.of(130.9418, 101.4684, 320);
        flowerToShoot = poseFactory.of(95.3135, 129.9457, -38.6349);
    }

    public Command autoRoutine() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(shooter::flywheelReady).withTimeout(AIM_TIMEOUT_MS),
                new TransferSequence(intake, gate),
                new IntakeFlowerDown(flower),
                new FollowPathCommand(follower, startToFlower()),
                new intakeOn(intake),
                new WaitCommand(1000),
                new FollowPathCommand(follower, flowerToShoot()),
                new intakeOff(intake),
                new IntakeFlowerUp(flower),
                new WaitUntilCommand(shooter::flywheelReady).withTimeout(AIM_TIMEOUT_MS),
                new TransferSequence(intake, gate),
                new InstantCommand(() -> shooter.setState(Shooter.State.OFF))
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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryData = new TelemetryData(telemetry);

        alliance = PosePersistency.getAlliance(Field.Alliance.BLUE);
    }

    @Override
    public void initialize_loop() {
        if (gamepad1.x) alliance = Field.Alliance.BLUE;
        if (gamepad1.b) alliance = Field.Alliance.RED;
        if (gamepad1.y) turret.zeroHere();

        turret.readEncoder();
        telemetryData.addData("Controls", "X = BLUE, B = RED, Y = zero turret here (point it forward)");
        telemetryData.addData("Alliance", alliance);
        telemetryData.addData("Turret deg", Math.toDegrees(turret.getAngle()));
        telemetryData.addData("Encoder fault", turret.isEncoderFault());
        telemetryData.update();
    }

    @Override
    public void preRun() {
        buildPoses(alliance);
        follower.setPose(start);
        follower.update();
        PosePersistency.saveAlliance(alliance);
        PosePersistency.savePose(start);
        turret.setAlliance(alliance);
        turret.setState(TurretMotor.State.AIM);
        shooter.setTargetRPM(ShooterConstants.TUNING_RPM);
        shooter.setState(Shooter.State.FIXED);
        schedule(autoRoutine());
    }

    @Override
    public void run() {
        super.run();
        follower.update();
        PosePersistency.savePose(follower.pose());

        telemetryData.addData("Alliance", alliance);
        telemetryData.addData("Pose", follower.pose());
        telemetryData.addData("Intake state", intake.getState());
        telemetryData.addData("Flower state", flower.getState());
        if (follower.currentPath() != null) {
            telemetryData.addData("Current path distance remaining", follower.distanceToEndpoint());
        }
        pollenSensor.addTelemetry(telemetryData);
        indicator.addTelemetry(telemetryData);
        gate.addTelemetry(telemetryData);
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
        return Paths.line(start, startToFlower).linear(start, startToFlower);
    }

    public Path flowerToShoot() {
        return Paths.line(startToFlower, flowerToShoot).reverseTangent();
    }
}
