package org.firstinspires.ftc.teamcode.subsystems.turret;

import static org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.*;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.HiveCells;
import org.firstinspires.ftc.teamcode.vision.CellObservation;
import org.firstinspires.ftc.teamcode.vision.Limelight;

public class Turret extends SubsystemBase {
    public enum VisionState {NONE, LOCKED, REJECTED, DOWN}

    private final CRServo servo;
    private final AnalogInput encoder;
    private final DcMotorEx shooter;
    private final Limelight vision;
    private final TurretHistory history;
    private final PIDFController controller = new PIDFController(kP, kI, kD, kF);
    private final InterpLUT shooterTable = new InterpLUT();

    private double requestedTarget = 0;
    private double commandedTarget = 0;
    private boolean reachable = true;
    private boolean limp = false;
    private boolean autoAim = false;

    private double rawDeg = 0;
    private boolean encoderFault = false;
    private double angle = 0;
    private double velocity = 0;
    private double power = 0;
    private long previousAngleNanos = -1;
    private double previousAngle = 0;

    private Pose botPose = new Pose(0, 0, 0);
    private double robotTurnRate = 0;
    private double robotVelX = 0;
    private double robotVelY = 0;
    private long previousPoseNanos = -1;

    private Field.Alliance alliance = null;
    private HiveCells.Cell upCell = null;
    private boolean upCellConfirmed = false;
    private long lastFlipNanos = -1;

    private double odomAngle = 0;
    private double odomDistance = Double.NaN;
    private double bias = 0;
    private double visionDistance = Double.NaN;
    private double distance = Double.NaN;
    private long lastVisionLockNanos = -1;
    private long previousAimNanos = -1;
    private VisionState visionState = VisionState.NONE;
    private CellObservation lastObservation = null;

    private boolean shooterEnabled = false;
    private double shooterTarget = 0;

    public Turret(HardwareMap hardwareMap, TurretHistory history) {
        this(hardwareMap, null, history);
    }

    public Turret(HardwareMap hardwareMap, Limelight vision, TurretHistory history) {
        this.vision = vision;
        this.history = history;
        servo = hardwareMap.get(CRServo.class, SERVO_NAME);
        encoder = hardwareMap.get(AnalogInput.class, ENCODER_NAME);
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        for (int i = 0; i < Math.min(SHOOTER_DIST_IN.length, SHOOTER_VEL.length); i++) {
            shooterTable.add(SHOOTER_DIST_IN[i], SHOOTER_VEL[i]);
        }
        shooterTable.createLUT();
    }

    public void updateBotPose(Pose pose) {
        long now = System.nanoTime();
        if (previousPoseNanos > 0) {
            double dt = (now - previousPoseNanos) * 1e-9;
            if (dt > 1e-4) {
                double rate = wrap(pose.heading() - botPose.heading()) / dt;
                robotTurnRate += VELOCITY_FILTER_ALPHA * (rate - robotTurnRate);
                robotVelX += VELOCITY_FILTER_ALPHA * ((pose.x() - botPose.x()) / dt - robotVelX);
                robotVelY += VELOCITY_FILTER_ALPHA * ((pose.y() - botPose.y()) / dt - robotVelY);
            }
        }
        previousPoseNanos = now;
        botPose = pose;
    }

    public void setTargetAngle(double radians) {
        requestedTarget = radians;
    }

    public void setLimp(boolean limp) {
        this.limp = limp;
    }

    public void setAutoAim(boolean autoAim) {
        this.autoAim = autoAim;
    }

    public void setAlliance(Field.Alliance alliance) {
        if (alliance != this.alliance) {
            upCell = null;
            upCellConfirmed = false;
            bias = 0;
        }
        this.alliance = alliance;
    }

    public void setUpCell(HiveCells.Cell cell) {
        upCell = cell;
        upCellConfirmed = false;
    }

    public void setShooterEnabled(boolean enabled) {
        shooterEnabled = enabled;
    }

    public void readEncoder() {
        rawDeg = encoder.getVoltage() / encoder.getMaxVoltage() * RAW_RANGE_DEG;
        encoderFault = Double.isNaN(rawDeg) || Math.abs(rawDeg - ZERO_RAW_DEG) > RAW_SANITY_MARGIN_DEG;
        if (!encoderFault) {
            angle = rawToAngle(rawDeg, ZERO_RAW_DEG, GEAR_RATIO, ENCODER_REVERSED);
        }
    }

    @Override
    public void periodic() {
        long now = System.nanoTime();
        if (autoAim) {
            updateAim(now);
        }
        updateTurret(now);
        updateShooter();
    }

    private void updateAim(long now) {
        double dt = previousAimNanos > 0 ? (now - previousAimNanos) * 1e-9 : 0;
        previousAimNanos = now;

        updateUpCell(now);
        if (upCell == null) {
            odomDistance = Double.NaN;
            distance = Double.NaN;
            visionState = VisionState.NONE;
            requestedTarget = 0;
            return;
        }

        Pose cellPose = upCell.pose(HiveCells.State.UP);
        double aimX = cellPose.x() + AIM_OFFSET_X_IN;
        double aimY = cellPose.y() + AIM_OFFSET_Y_IN;

        double[] pivotNow = AimMath.pivot(botPose.x(), botPose.y(), botPose.heading(), TURRET_FWD, TURRET_LEFT);
        double odomAngleNow = AimMath.odomAngle(pivotNow[0], pivotNow[1], botPose.heading(), aimX, aimY);

        double[] pivotLead = AimMath.pivot(botPose.x() + robotVelX * SHOT_LEAD_S, botPose.y() + robotVelY * SHOT_LEAD_S,
                botPose.heading(), TURRET_FWD, TURRET_LEFT);
        odomAngle = AimMath.odomAngle(pivotLead[0], pivotLead[1], botPose.heading(), aimX, aimY);
        odomDistance = Math.hypot(aimX - pivotLead[0], aimY - pivotLead[1]);

        CellObservation obs = vision != null && vision.hasNewFrame() ? vision.getObservation(upCell) : null;
        if (obs != null) {
            lastObservation = obs;
            if (obs.state == HiveCells.State.DOWN) {
                visionState = VisionState.DOWN;
            } else {
                double visionAngle = AimMath.visionAngle(obs.turretAngleAtCapture, obs.bearing,
                        botPose.heading(), obs.headingAtCapture);
                double error = wrap(visionAngle - odomAngleNow);
                if (Math.abs(error) > Math.toRadians(VISION_MAX_DISAGREE_DEG)) {
                    visionState = VisionState.REJECTED;
                } else {
                    bias = AimMath.updateBias(bias, error, BIAS_ALPHA);
                    visionDistance = obs.distance;
                    lastVisionLockNanos = now;
                    visionState = VisionState.LOCKED;
                }
            }
        }

        boolean recentVision = hasRecentVision(now);
        if (!recentVision) {
            bias = AimMath.decayBias(bias, BIAS_DECAY_PER_S, dt);
            if (obs == null) visionState = VisionState.NONE;
        }
        distance = recentVision ? visionDistance : odomDistance;
        requestedTarget = wrap(odomAngle + bias);
    }

    private void updateUpCell(long now) {
        if (alliance == null || vision == null || !vision.hasNewFrame()) return;
        HiveCells.Cell seenUp = null;
        for (HiveCells.Cell cell : HiveCells.Cell.values()) {
            if (cell.alliance != alliance) continue;
            CellObservation obs = vision.getObservation(cell);
            if (obs == null) continue;
            if (obs.state == HiveCells.State.UP) seenUp = cell;
            else if (obs.state == HiveCells.State.DOWN) seenUp = cell.partner();
            if (seenUp != null) break;
        }
        if (seenUp == null) return;
        if (upCell != null && seenUp != upCell) {
            lastFlipNanos = now;
            bias = 0;
            lastVisionLockNanos = -1;
        }
        upCell = seenUp;
        upCellConfirmed = true;
    }

    private void updateTurret(long now) {
        servo.setDirection(SERVO_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        readEncoder();
        if (encoderFault) {
            setPower(0);
            previousAngleNanos = -1;
            return;
        }

        if (previousAngleNanos > 0) {
            double dt = (now - previousAngleNanos) * 1e-9;
            if (dt > 1e-4) {
                velocity += VELOCITY_FILTER_ALPHA * ((angle - previousAngle) / dt - velocity);
            }
        }
        previousAngleNanos = now;
        previousAngle = angle;
        history.add(now, angle, botPose.x(), botPose.y(), botPose.heading());

        double min = Math.toRadians(MIN_ANGLE_DEG);
        double max = Math.toRadians(MAX_ANGLE_DEG);
        double equivalent = reachableEquivalent(requestedTarget, angle, min, max);
        reachable = !Double.isNaN(equivalent);
        commandedTarget = reachable ? equivalent : nearestLimit(requestedTarget, min, max);

        controller.setPIDF(kP, kI, kD, kF);
        double error = commandedTarget - angle;
        double output = controller.calculate(angle, commandedTarget);
        if (Math.abs(error) > Math.toRadians(DEADBAND_DEG)) {
            output += kS * Math.signum(error);
        }
        output += kTurnFF * -robotTurnRate;
        output = Range.clip(output, -MAX_POWER, MAX_POWER);

        if (limp || (angle >= max && output > 0) || (angle <= min && output < 0)) {
            output = 0;
        }
        setPower(output);
    }

    private void updateShooter() {
        shooter.setDirection(SHOOTER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        if (TUNING_SHOOTER) {
            shooterTarget = TUNING_VELOCITY;
        } else {
            shooterTarget = Double.isNaN(distance) ? 0 : shooterTable.get(distance);
        }
        if (shooterEnabled && shooterTarget > 0) {
            shooter.setVelocity(shooterTarget);
        } else {
            shooter.setPower(0);
        }
    }

    private void setPower(double output) {
        power = output;
        servo.setPower(output);
    }

    private boolean hasRecentVision(long now) {
        return lastVisionLockNanos > 0 && (now - lastVisionLockNanos) * 1e-6 <= VISION_LOCK_VALID_MS;
    }

    public AimMath.Status getStatus() {
        long now = System.nanoTime();
        boolean justFlipped = lastFlipNanos > 0 && (now - lastFlipNanos) * 1e-6 < HIVE_FLIP_COOLDOWN_MS;
        boolean shooterReady = shooterEnabled && shooterTarget > 0
                && Math.abs(getShooterVelocity() - shooterTarget) <= SHOOTER_TOLERANCE;
        return AimMath.status(upCell != null, justFlipped, distance, MIN_SHOT_DIST, MAX_SHOT_DIST,
                reachable, isSettled(), shooterReady, hasRecentVision(now), ALLOW_ODOMETRY_ONLY_SHOTS);
    }

    public boolean okToShoot() {
        return autoAim && getStatus() == AimMath.Status.OK;
    }

    static double rawToAngle(double rawDeg, double zeroRawDeg, double gearRatio, boolean reversed) {
        double turretDeg = (rawDeg - zeroRawDeg) / gearRatio;
        return Math.toRadians(reversed ? -turretDeg : turretDeg);
    }

    static double reachableEquivalent(double target, double current, double min, double max) {
        double best = Double.NaN;
        double base = wrap(target);
        for (int k = -1; k <= 1; k++) {
            double candidate = base + k * 2 * Math.PI;
            if (candidate < min || candidate > max) continue;
            if (Double.isNaN(best) || Math.abs(candidate - current) < Math.abs(best - current)) {
                best = candidate;
            }
        }
        return best;
    }

    static double nearestLimit(double target, double min, double max) {
        return Math.abs(wrap(target - min)) <= Math.abs(wrap(target - max)) ? min : max;
    }

    static double wrap(double radians) {
        return AimMath.wrap(radians);
    }

    public double getAngle() {
        return angle;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getRequestedTarget() {
        return requestedTarget;
    }

    public double getCommandedTarget() {
        return commandedTarget;
    }

    public boolean isReachable() {
        return reachable;
    }

    public boolean isEncoderFault() {
        return encoderFault;
    }

    public boolean isSettled() {
        return !encoderFault
                && Math.abs(commandedTarget - angle) < Math.toRadians(AIM_TOLERANCE_DEG)
                && Math.abs(velocity) < Math.toRadians(SETTLED_VEL_DEG_S);
    }

    public double getRawDeg() {
        return rawDeg;
    }

    public double getPower() {
        return power;
    }

    public double getRobotTurnRate() {
        return robotTurnRate;
    }

    public Pose getBotPose() {
        return botPose;
    }

    public Field.Alliance getAlliance() {
        return alliance;
    }

    public HiveCells.Cell getUpCell() {
        return upCell;
    }

    public boolean isUpCellConfirmed() {
        return upCellConfirmed;
    }

    public double getOdomAngle() {
        return odomAngle;
    }

    public double getBias() {
        return bias;
    }

    public double getDistance() {
        return distance;
    }

    public VisionState getVisionState() {
        return visionState;
    }

    public CellObservation getLastObservation() {
        return lastObservation;
    }

    public double getShooterTarget() {
        return shooterTarget;
    }

    public double getShooterVelocity() {
        return shooter.getVelocity();
    }
}
