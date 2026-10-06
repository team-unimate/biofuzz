package org.firstinspires.ftc.teamcode.subsystems.turret;

import static org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.*;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.TauraServo;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.HiveCells;
import org.firstinspires.ftc.teamcode.util.Clock;
import org.firstinspires.ftc.teamcode.vision.CellObservation;
import org.firstinspires.ftc.teamcode.vision.Limelight;

public class Turret extends SubsystemBase {
    public enum VisionState {NONE, LOCKED, REJECTED, DOWN}

    private final TauraServo servo;
    private final AnalogInput encoder;
    private final DcMotorEx shooter;
    private final Limelight vision;
    private final TurretHistory history;
    private final PIDFController turretController = new PIDFController(kP, kI, kD, kF);
    private final InterpLUT velocityInterpolation = new InterpLUT();

    private Pose botPose = new Pose(0, 0, 0);

    private boolean autoAim = false;
    private boolean limp = false;
    private double manualAngle = 0;

    private double rawDeg = 0;
    private boolean encoderFault = false;
    private double currentAngle = 0;
    private double previousAngle = 0;
    private double previousAngleTime = -1;
    private double velocity = 0;
    private double power = 0;

    private Field.Alliance alliance = null;
    private HiveCells.Cell upCell = null;
    private boolean upCellConfirmed = false;
    private double lastFlipTime = -1;
    private double goalAngleBC = 0;
    private double targetAngle = 0;
    private boolean okToShoot = true;
    private double distance = Double.NaN;

    private double visionOffset = 0;
    private double lastVisionLockTime = -1;
    private VisionState visionState = VisionState.NONE;
    private CellObservation lastObservation = null;

    private boolean shooterEnabled = false;
    private double targetVelocity = 0;
    private double shooterOverride = Double.NaN;

    public Turret(HardwareMap hardwareMap, TurretHistory history) {
        this(hardwareMap, null, history);
    }

    public Turret(HardwareMap hardwareMap, Limelight vision, TurretHistory history) {
        this.vision = vision;
        this.history = history;
        servo = new TauraServo(hardwareMap.get(Servo.class, SERVO_NAME));
        encoder = hardwareMap.get(AnalogInput.class, ENCODER_NAME);
        servo.setAnalogFeedbackSensor(encoder);
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        buildVelocityTable();
    }

    public void updateBotPose(Pose pose) {
        this.botPose = pose;
    }

    public void setTargetAngle(double radians) {
        manualAngle = radians;
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
            visionOffset = 0;
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

    public void setShooterOverride(double ticksPerSecond) {
        shooterOverride = ticksPerSecond;
    }

    public void clearShooterOverride() {
        shooterOverride = Double.NaN;
    }

    public void readEncoder() {
        rawDeg = servo.getRawPositionInDegrees();
        encoderFault = Double.isNaN(rawDeg) || Math.abs(rawDeg - ZERO_RAW_DEG) > RAW_SANITY_MARGIN_DEG;
        if (!encoderFault) {
            currentAngle = rawToAngle(rawDeg, ZERO_RAW_DEG, GEAR_RATIO, ENCODER_REVERSED);
        }
    }

    @Override
    public void periodic() {
        turretController.setPIDF(kP, kI, kD, kF);

        readEncoder();
        updateVelocity();
        history.add(Clock.seconds(), currentAngle, botPose.x(), botPose.y(), botPose.heading());

        if (autoAim) {
            updateUpCell();
            updateAim();
        } else {
            targetAngle = manualAngle;
            okToShoot = true;
        }
        updateTurret();
        updateShooter();
    }

    private void updateAim() {
        if (upCell == null) {
            distance = Double.NaN;
            visionState = VisionState.NONE;
            targetAngle = 0;
            return;
        }

        Pose goalPose = upCell.pose(HiveCells.State.UP);
        double goalX = goalPose.x() + AIM_OFFSET_X_IN;
        double goalY = goalPose.y() + AIM_OFFSET_Y_IN;

        double heading = botPose.heading();
        double turretX = botPose.x() + TURRET_FWD * Math.cos(heading) - TURRET_LEFT * Math.sin(heading);
        double turretY = botPose.y() + TURRET_FWD * Math.sin(heading) + TURRET_LEFT * Math.cos(heading);

        double goalAngleFC = Math.atan2(goalY - turretY, goalX - turretX);
        goalAngleBC = normalizeAngle(goalAngleFC - heading);

        updateVisionOffset();

        double sideOffset = alliance == Field.Alliance.RED ? RED_OFFSET_DEG : BLUE_OFFSET_DEG;
        double goalAngle = normalizeAngle(goalAngleBC + Math.toRadians(sideOffset) + visionOffset);

        double softLimit = Math.toRadians(TURRET_SOFT_LIMIT_DEG);
        double goalAngleCorrected = Range.clip(goalAngle, -softLimit, softLimit);

        okToShoot = goalAngle == goalAngleCorrected;
        if (!okToShoot) {
            goalAngleCorrected = 0;
        }

        targetAngle = goalAngleCorrected;
        distance = Math.hypot(goalX - turretX, goalY - turretY);
    }


    private void updateVisionOffset() {
        double now = Clock.seconds();
        CellObservation obs = vision != null && vision.hasNewFrame() ? vision.getObservation(upCell) : null;
        if (obs != null) {
            lastObservation = obs;
            if (obs.state == HiveCells.State.DOWN) {
                visionState = VisionState.DOWN;
            } else {

                double turnSinceCapture = normalizeAngle(botPose.heading() - obs.headingAtCapture);
                double visionAngle = normalizeAngle(obs.turretAngleAtCapture + obs.bearing - turnSinceCapture);
                double error = normalizeAngle(visionAngle - goalAngleBC);

                if (Math.abs(error) > Math.toRadians(VISION_MAX_DISAGREE_DEG)) {
                    visionState = VisionState.REJECTED;
                } else {
                    visionOffset += VISION_OFFSET_GAIN * (error - visionOffset);
                    lastVisionLockTime = now;
                    visionState = VisionState.LOCKED;
                }
            }
        }

        if (!hasRecentVision(now)) {
            visionOffset *= VISION_OFFSET_FADE;
            if (obs == null) visionState = VisionState.NONE;
        }
    }

    private void updateUpCell() {
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
            lastFlipTime = Clock.seconds();
            visionOffset = 0;
            lastVisionLockTime = -1;
        }
        upCell = seenUp;
        upCellConfirmed = true;
    }

    private void updateTurret() {
        servo.setDirection(SERVO_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        if (encoderFault || limp) {
            setPower(0);
            return;
        }

        turretController.setSetPoint(targetAngle);
        double power = turretController.calculate(currentAngle);

        double error = targetAngle - currentAngle;
        if (Math.abs(error) > Math.toRadians(DEADBAND_DEG)) {
            power += kS * Math.signum(error);
        }

        double hardLimit = Math.toRadians(TURRET_HARD_LIMIT_DEG);
        boolean positiveLimit = currentAngle >= hardLimit;
        boolean negativeLimit = currentAngle <= -hardLimit;

        if (positiveLimit && power > 0) {
            power = 0;
        }

        if (negativeLimit && power < 0) {
            power = 0;
        }

        setPower(Range.clip(power, -MAX_POWER, MAX_POWER));
    }

    private void updateVelocity() {
        double now = Clock.seconds();
        if (encoderFault) {
            previousAngleTime = -1;
            return;
        }
        if (previousAngleTime >= 0 && now > previousAngleTime) {
            double measured = (currentAngle - previousAngle) / (now - previousAngleTime);
            velocity += VELOCITY_FILTER_ALPHA * (measured - velocity);
        }
        previousAngle = currentAngle;
        previousAngleTime = now;
    }

    private void updateShooter() {
        shooter.setDirection(SHOOTER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        if (!Double.isNaN(shooterOverride)) {
            targetVelocity = shooterOverride;
        } else if (TUNING_SHOOTER) {
            targetVelocity = TUNING_VELOCITY;
        } else if (Double.isNaN(distance)) {
            targetVelocity = 0;
        } else {
            double clippedDistance = Range.clip(distance, SHOOTER_DIST_IN[0], SHOOTER_DIST_IN[SHOOTER_DIST_IN.length - 1]);
            targetVelocity = velocityInterpolation.get(clippedDistance);
        }

        if (shooterEnabled && targetVelocity > 0) {
            shooter.setVelocity(targetVelocity);
        } else {
            shooter.setPower(0);
        }
    }

    private void buildVelocityTable() {
        for (int i = 0; i < Math.min(SHOOTER_DIST_IN.length, SHOOTER_VEL.length); i++) {
            velocityInterpolation.add(SHOOTER_DIST_IN[i], SHOOTER_VEL[i]);
        }
        velocityInterpolation.createLUT();
    }

    private void setPower(double output) {
        power = output;
        servo.setPosition(0.5 + 0.5 * output);
    }

    private boolean hasRecentVision(double now) {
        return lastVisionLockTime >= 0 && (now - lastVisionLockTime) * 1000 <= VISION_LOCK_VALID_MS;
    }

    public AimMath.Status getStatus() {
        double now = Clock.seconds();
        boolean justFlipped = lastFlipTime >= 0 && (now - lastFlipTime) * 1000 < HIVE_FLIP_COOLDOWN_MS;
        boolean shooterReady = shooterEnabled && targetVelocity > 0
                && Math.abs(getShooterVelocity() - targetVelocity) <= SHOOTER_TOLERANCE;
        return AimMath.status(upCell != null, justFlipped, distance, MIN_SHOT_DIST, MAX_SHOT_DIST,
                okToShoot, isSettled(), shooterReady, hasRecentVision(now), ALLOW_ODOMETRY_ONLY_SHOTS);
    }

    public boolean okToShoot() {
        return autoAim && getStatus() == AimMath.Status.OK;
    }

    static double rawToAngle(double rawDeg, double zeroRawDeg, double gearRatio, boolean reversed) {
        double turretDeg = (rawDeg - zeroRawDeg) / gearRatio;
        return Math.toRadians(reversed ? -turretDeg : turretDeg);
    }

    static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public double getAngle() {
        return currentAngle;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getCommandedTarget() {
        return targetAngle;
    }

    public boolean isReachable() {
        return okToShoot;
    }

    public boolean isEncoderFault() {
        return encoderFault;
    }

    public boolean isSettled() {
        return !encoderFault
                && Math.abs(targetAngle - currentAngle) < Math.toRadians(AIM_TOLERANCE_DEG)
                && Math.abs(velocity) < Math.toRadians(SETTLED_VEL_DEG_S);
    }

    public double getRawDeg() {
        return rawDeg;
    }

    public double getPower() {
        return power;
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
        return goalAngleBC;
    }

    public double getBias() {
        return visionOffset;
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
        return targetVelocity;
    }

    public double getShooterVelocity() {
        return shooter.getVelocity();
    }
}
