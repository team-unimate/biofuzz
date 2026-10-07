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
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.TauraServo;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.HiveCells;
import org.firstinspires.ftc.teamcode.vision.CellObservation;
import org.firstinspires.ftc.teamcode.vision.Limelight;

public class Turret extends SubsystemBase {
    private final TauraServo servo;
    private final DcMotorEx shooter;
    private final Limelight vision;
    private final TurretHistory history;
    private final PIDFController controller = new PIDFController(kP, kI, kD, kF);
    private final InterpLUT shooterTable = new InterpLUT();

    private Pose botPose = new Pose(0, 0, 0);
    private Field.Alliance alliance = null;
    private HiveCells.Cell cell = null;

    private double manualAngle = Double.NaN;
    private double rawDeg = 0;
    private boolean encoderFault = false;
    private double angle = 0;
    private double velocity = 0;
    private double power = 0;
    private long previousNanos = -1;
    private double previousAngle = 0;

    private double odomAngle = 0;
    private double bias = 0;
    private double target = 0;
    private double setpoint = 0;
    private boolean inRange = true;
    private double distance = Double.NaN;
    private double visionDistance = Double.NaN;
    private long lastSeenNanos = -1;
    private boolean seen = false;

    private boolean shooterEnabled = false;
    private double shooterTarget = 0;
    private double shooterOverride = Double.NaN;

    public Turret(HardwareMap hardwareMap, TurretHistory history) {
        this(hardwareMap, null, history);
    }

    public Turret(HardwareMap hardwareMap, Limelight vision, TurretHistory history) {
        this.vision = vision;
        this.history = history;
        servo = new TauraServo(hardwareMap.get(Servo.class, SERVO_NAME));
        servo.setAnalogFeedbackSensor(hardwareMap.get(AnalogInput.class, ENCODER_NAME));
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        for (int i = 0; i < Math.min(SHOOTER_DIST_IN.length, SHOOTER_VEL.length); i++) {
            shooterTable.add(SHOOTER_DIST_IN[i], SHOOTER_VEL[i]);
        }
        shooterTable.createLUT();
    }

    public void updateBotPose(Pose pose) {
        botPose = pose;
    }

    public void setAlliance(Field.Alliance alliance) {
        if (alliance != this.alliance) {
            bias = 0;
            lastSeenNanos = -1;
        }
        this.alliance = alliance;
    }

    public void setManualAngle(double radians) {
        manualAngle = radians;
    }

    public void clearManualAngle() {
        manualAngle = Double.NaN;
    }

    public void setShooterEnabled(boolean enabled) {
        shooterEnabled = enabled;
    }

    public void setShooterOverride(double ticksPerSecond) {
        shooterOverride = ticksPerSecond;
    }

    public void readEncoder() {
        rawDeg = servo.getRawPositionInDegrees();
        encoderFault = Double.isNaN(rawDeg) || Math.abs(rawDeg - ZERO_RAW_DEG) > RAW_SANITY_MARGIN_DEG;
        if (!encoderFault) {
            angle = rawToAngle(rawDeg, ZERO_RAW_DEG, GEAR_RATIO, ENCODER_REVERSED);
        }
    }

    @Override
    public void periodic() {
        long now = System.nanoTime();
        updateAim(now);
        updateTurret(now);
        updateShooter();
    }

    private void updateAim(long now) {
        double dt = previousNanos > 0 ? (now - previousNanos) * 1e-9 : 0;
        HiveCells.Cell nextCell = alliance == null ? null : HiveCells.forRobot(alliance, botPose);
        if (nextCell != cell) {
            cell = nextCell;
            bias = 0;
            lastSeenNanos = -1;
        }
        if (cell == null) {
            target = 0;
            distance = Double.NaN;
            seen = false;
            return;
        }

        double[] pivot = pivot(botPose.x(), botPose.y(), botPose.heading(), TURRET_FWD, TURRET_LEFT);
        odomAngle = odomAngle(pivot[0], pivot[1], botPose.heading(), cell.pose.x(), cell.pose.y());
        double odomDistance = Math.hypot(cell.pose.x() - pivot[0], cell.pose.y() - pivot[1]);

        CellObservation obs = vision != null && vision.hasNewFrame() ? vision.getObservation(cell) : null;
        if (obs != null) {
            double visionAngle = visionAngle(obs.turretAngleAtCapture, obs.bearing, botPose.heading(), obs.headingAtCapture);
            double error = wrap(visionAngle - odomAngle);
            if (Math.abs(error) <= Math.toRadians(VISION_MAX_DISAGREE_DEG)) {
                bias = error;
                visionDistance = obs.distance;
                lastSeenNanos = now;
            }
        }

        seen = lastSeenNanos > 0 && (now - lastSeenNanos) * 1e-6 <= VISION_LOCK_VALID_MS;
        if (!seen) bias *= Math.exp(-BIAS_DECAY_PER_S * dt);
        target = wrap(odomAngle + bias);
        distance = seen ? visionDistance : odomDistance;
    }

    private void updateTurret(long now) {
        servo.setDirection(SERVO_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

        readEncoder();
        if (encoderFault) {
            setPower(0);
            previousNanos = -1;
            return;
        }

        if (previousNanos > 0) {
            double dt = (now - previousNanos) * 1e-9;
            if (dt > 1e-4) velocity += VELOCITY_FILTER_ALPHA * ((angle - previousAngle) / dt - velocity);
        }
        previousNanos = now;
        previousAngle = angle;
        history.add(now, angle, botPose.x(), botPose.y(), botPose.heading());

        double min = Math.toRadians(MIN_ANGLE_DEG);
        double max = Math.toRadians(MAX_ANGLE_DEG);
        double goal = Double.isNaN(manualAngle) ? target : manualAngle;
        inRange = goal >= min && goal <= max;
        setpoint = inRange ? goal : 0;

        controller.setPIDF(kP, kI, kD, kF);
        double error = setpoint - angle;
        double output = controller.calculate(angle, setpoint);
        if (Math.abs(error) > Math.toRadians(DEADBAND_DEG)) output += kS * Math.signum(error);
        output = Range.clip(output, -MAX_POWER, MAX_POWER);

        if ((angle >= max && output > 0) || (angle <= min && output < 0)) output = 0;
        setPower(output);
    }

    private void updateShooter() {
        shooter.setDirection(SHOOTER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        if (!Double.isNaN(shooterOverride)) {
            shooterTarget = shooterOverride;
        } else if (TUNING_SHOOTER) {
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
        servo.setPosition(0.5 + 0.5 * output);
    }

    public boolean isTooClose() {
        return seen && visionDistance < MIN_SHOT_DIST;
    }

    public boolean isSettled() {
        return !encoderFault
                && Math.abs(setpoint - angle) < Math.toRadians(AIM_TOLERANCE_DEG)
                && Math.abs(velocity) < Math.toRadians(SETTLED_VEL_DEG_S);
    }

    public boolean isShooterReady() {
        return shooterEnabled && shooterTarget > 0
                && Math.abs(shooter.getVelocity() - shooterTarget) <= SHOOTER_TOLERANCE;
    }

    public boolean okToShoot() {
        return cell != null && Double.isNaN(manualAngle) && inRange && isSettled()
                && isShooterReady() && !isTooClose() && distance <= MAX_SHOT_DIST
                && (seen || ALLOW_ODOMETRY_ONLY_SHOTS);
    }

    public void addTelemetry(TelemetryData telemetry) {
        telemetry.addData("Turret OK to shoot", okToShoot());
        telemetry.addData("Turret cell", cell == null ? "none" : cell);
        telemetry.addData("Turret tags seen", seen);
        telemetry.addData("Turret too close", isTooClose());
        telemetry.addData("Turret in range", inRange);
        telemetry.addData("Turret settled", isSettled());
        telemetry.addData("Turret distance in", distance);
        telemetry.addData("Turret odom deg", Math.toDegrees(odomAngle));
        telemetry.addData("Turret bias deg", Math.toDegrees(bias));
        telemetry.addData("Turret target deg", Math.toDegrees(target));
        telemetry.addData("Turret angle deg", Math.toDegrees(angle));
        telemetry.addData("Turret raw deg", rawDeg);
        telemetry.addData("Turret encoder fault", encoderFault);
        telemetry.addData("Shooter target", shooterTarget);
        telemetry.addData("Shooter velocity", shooter.getVelocity());
    }

    static double rawToAngle(double rawDeg, double zeroRawDeg, double gearRatio, boolean reversed) {
        double turretDeg = (rawDeg - zeroRawDeg) / gearRatio;
        return Math.toRadians(reversed ? -turretDeg : turretDeg);
    }

    static double[] pivot(double x, double y, double heading, double pivotForward, double pivotLeft) {
        double c = Math.cos(heading), s = Math.sin(heading);
        return new double[]{x + pivotForward * c - pivotLeft * s, y + pivotForward * s + pivotLeft * c};
    }

    static double odomAngle(double pivotX, double pivotY, double heading, double aimX, double aimY) {
        return wrap(Math.atan2(aimY - pivotY, aimX - pivotX) - heading);
    }

    static double visionAngle(double turretAtCapture, double bearing, double headingNow, double headingAtCapture) {
        return wrap(turretAtCapture + bearing - wrap(headingNow - headingAtCapture));
    }

    public static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    public double getAngle() {
        return angle;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getPower() {
        return power;
    }

    public double getRawDeg() {
        return rawDeg;
    }

    public boolean isEncoderFault() {
        return encoderFault;
    }

    public boolean isInRange() {
        return inRange;
    }

    public boolean isSeen() {
        return seen;
    }

    public double getDistance() {
        return distance;
    }

    public HiveCells.Cell getCell() {
        return cell;
    }

    public double getShooterVelocity() {
        return shooter.getVelocity();
    }
}
