package org.firstinspires.ftc.teamcode.subsystems.turret;

import static org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.*;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

/**
 * Turret driven by one stemOS Taura in continuous mode, with the Taura's analog feedback used as an
 * absolute encoder (see TURRET.md, "Hardware" and steps 1 and 6).
 * <p>
 * The turret's whole range fits inside one sensor turn, so the angle is read directly from the raw
 * reading every loop: no zeroing at init, no unwrapping. All angles are radians, robot-relative,
 * 0 = forward, CCW positive.
 * <p>
 * Loop order in OpModes: {@code follower.update()} -> {@link #updateBotPose(Pose)} -> {@code super.run()}.
 */
public class Turret extends SubsystemBase {
    private final CRServo servo;
    private final AnalogInput encoder;
    private final PIDFController controller = new PIDFController(kP, kI, kD, kF);

    private double requestedTarget = 0;
    private double commandedTarget = 0;
    private boolean reachable = true;
    private boolean limp = false;

    private double rawDeg = 0;
    private boolean encoderFault = false;
    private double angle = 0;
    private double velocity = 0;
    private double power = 0;
    private long previousAngleNanos = -1;
    private double previousAngle = 0;

    private Pose botPose = new Pose(0, 0, 0);
    private double robotTurnRate = 0;
    private long previousPoseNanos = -1;

    public Turret(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, SERVO_NAME);
        encoder = hardwareMap.get(AnalogInput.class, ENCODER_NAME);
    }

    /** Robot pose from the follower, once per loop before the scheduler runs. Also measures robot turn rate. */
    public void updateBotPose(Pose pose) {
        long now = System.nanoTime();
        if (previousPoseNanos > 0) {
            double dt = (now - previousPoseNanos) * 1e-9;
            if (dt > 1e-4) {
                double rate = wrap(pose.heading() - botPose.heading()) / dt;
                robotTurnRate += VELOCITY_FILTER_ALPHA * (rate - robotTurnRate);
            }
        }
        previousPoseNanos = now;
        botPose = pose;
    }

    /** Robot-relative target angle (radians). Any angle is accepted; see {@link #isReachable()}. */
    public void setTargetAngle(double radians) {
        requestedTarget = radians;
    }

    /** When true the turret gets zero power (so it can be turned by hand); the encoder is still read. */
    public void setLimp(boolean limp) {
        this.limp = limp;
    }

    /**
     * Reads the encoder only, without driving the servo. Safe to call during init (robots must not
     * move before start). {@link #periodic()} calls it too.
     */
    public void readEncoder() {
        rawDeg = encoder.getVoltage() / encoder.getMaxVoltage() * RAW_RANGE_DEG;
        encoderFault = Double.isNaN(rawDeg) || Math.abs(rawDeg - ZERO_RAW_DEG) > RAW_SANITY_MARGIN_DEG;
        if (!encoderFault) {
            angle = rawToAngle(rawDeg, ZERO_RAW_DEG, GEAR_RATIO, ENCODER_REVERSED);
        }
    }

    @Override
    public void periodic() {
        servo.setDirection(SERVO_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        readEncoder();
        if (encoderFault) {
            setPower(0);
            previousAngleNanos = -1;
            return;
        }

        long now = System.nanoTime();
        if (previousAngleNanos > 0) {
            double dt = (now - previousAngleNanos) * 1e-9;
            if (dt > 1e-4) {
                velocity += VELOCITY_FILTER_ALPHA * ((angle - previousAngle) / dt - velocity);
            }
        }
        previousAngleNanos = now;
        previousAngle = angle;

        double min = Math.toRadians(MIN_ANGLE_DEG);
        double max = Math.toRadians(MAX_ANGLE_DEG);
        double equivalent = reachableEquivalent(requestedTarget, angle, min, max);
        reachable = !Double.isNaN(equivalent);
        commandedTarget = reachable ? equivalent : nearestLimit(requestedTarget, min, max);

        // Re-applied every loop so dashboard edits take effect immediately.
        controller.setPIDF(kP, kI, kD, kF);
        double error = commandedTarget - angle;
        double output = controller.calculate(angle, commandedTarget);
        if (Math.abs(error) > Math.toRadians(DEADBAND_DEG)) {
            output += kS * Math.signum(error);
        }
        output += kTurnFF * -robotTurnRate;
        output = Range.clip(output, -MAX_POWER, MAX_POWER);

        // Never push past the soft limits.
        if (limp || (angle >= max && output > 0) || (angle <= min && output < 0)) {
            output = 0;
        }
        setPower(output);
    }

    private void setPower(double output) {
        power = output;
        // CRServo power p is sent as servo position 0.5 + 0.5 * p.
        servo.setPower(output);
    }

    /** Raw sensor degrees -> turret radians. */
    static double rawToAngle(double rawDeg, double zeroRawDeg, double gearRatio, boolean reversed) {
        double turretDeg = (rawDeg - zeroRawDeg) / gearRatio;
        return Math.toRadians(reversed ? -turretDeg : turretDeg);
    }

    /** The equivalent of {@code target} (+-2pi) inside [min, max] closest to {@code current}, or NaN if none fits. */
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

    /** The soft limit angularly closest to an unreachable target. */
    static double nearestLimit(double target, double min, double max) {
        return Math.abs(wrap(target - min)) <= Math.abs(wrap(target - max)) ? min : max;
    }

    /** Wrap to -pi..pi. */
    static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    public double getAngle() {
        return angle;
    }

    /** Turret angular velocity, rad/s, filtered. */
    public double getVelocity() {
        return velocity;
    }

    public double getRequestedTarget() {
        return requestedTarget;
    }

    /** What the controller is actually driving to (the requested target, or a soft limit if out of range). */
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

    /** Robot turn rate, rad/s, filtered, measured from {@link #updateBotPose(Pose)}. */
    public double getRobotTurnRate() {
        return robotTurnRate;
    }

    public Pose getBotPose() {
        return botPose;
    }
}
