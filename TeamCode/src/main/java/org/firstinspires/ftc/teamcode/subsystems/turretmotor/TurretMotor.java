package org.firstinspires.ftc.teamcode.subsystems.turretmotor;

import static org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMotorConstants.*;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.HiveCells;
import org.firstinspires.ftc.teamcode.util.PosePersistency;
import org.firstinspires.ftc.teamcode.vision.Limelight;

import java.util.function.Supplier;

public class TurretMotor extends SubsystemBase {
    public enum State {
        STALL, MANUAL, AIM, IDLE
    }

    private final DcMotorEx motor;
    private final TurretHistory history;
    private final Supplier<Pose> poseSupplier;
    private final TurretAim aim;
    private final TurretController controller = new TurretController();
    private double zeroTicks;

    private State state = State.IDLE;
    private State requestedState = State.IDLE;
    private boolean recovering = false;
    private double stallTime = Double.NaN;
    private double manualAngle = 0;
    private double lastTime = Double.NaN;

    private double ticks = 0;
    private double angle = 0;
    private double velocity = 0;
    private boolean encoderFault = false;
    private double setpoint = 0;
    private boolean inRange = true;
    private double power = 0;

    public TurretMotor(HardwareMap hardwareMap, Limelight vision, TurretHistory history, Supplier<Pose> poseSupplier) {
        this.history = history;
        this.poseSupplier = poseSupplier;
        this.aim = new TurretAim(vision);
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setDirection(MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zeroTicks = PosePersistency.getTurretMotorZero(motor.getCurrentPosition());
        PosePersistency.saveTurretMotorZero(zeroTicks);
    }

    public void setAlliance(Field.Alliance alliance) {
        aim.setAlliance(alliance);
    }

    public void setState(State state) {
        requestedState = state;
    }

    public void setManualAngle(double radians) {
        manualAngle = radians;
        setState(State.MANUAL);
    }

    public void zeroHere() {
        zeroTicks = motor.getCurrentPosition();
        PosePersistency.saveTurretMotorZero(zeroTicks);
        readEncoder();
    }

    public void readEncoder() {
        ticks = motor.getCurrentPosition() - zeroTicks;
        angle = ticksToAngle(ticks);
        encoderFault = angle < Math.toRadians(MOTOR_MIN_ANGLE_DEG - MOTOR_SANITY_MARGIN_DEG)
                || angle > Math.toRadians(MOTOR_MAX_ANGLE_DEG + MOTOR_SANITY_MARGIN_DEG);
    }

    @Override
    public void periodic() {
        double now = history.now();
        double dt = Double.isNaN(lastTime) ? 0 : now - lastTime;
        lastTime = now;
        Pose robot = poseSupplier.get();

        aim.update(robot, now, dt);

        readEncoder();
        if (encoderFault) {
            setPower(0);
            return;
        }
        velocity += MOTOR_VELOCITY_FILTER_ALPHA * (ticksToAngle(motor.getVelocity()) - velocity);
        history.add(now, angle, robot.x(), robot.y(), robot.heading());

        if (state == State.STALL) {
            holdStill();
            if (requestedState != State.STALL && now - stallTime >= MOTOR_STALL_COOLDOWN_S) {
                recovering = true;
                changeState(State.IDLE);
            }
            return;
        }
        if (!recovering) changeState(nextState());
        if (state == State.STALL) {
            holdStill();
            return;
        }
        drive(now, dt);
    }

    private State nextState() {
        boolean canAim = aim.hasCell() && TurretController.isWithinLimits(aim.getTarget());
        if (requestedState == State.AIM && !canAim) return State.IDLE;
        return requestedState;
    }

    private void changeState(State newState) {
        if (newState == state) return;
        if (newState == State.STALL) stallTime = history.now();
        controller.reset();
        state = newState;
    }

    private double goalAngle() {
        switch (state) {
            case AIM:
                return aim.getTarget();
            case MANUAL:
                return manualAngle;
            default:
                return Math.toRadians(IDLE_ANGLE_DEG);
        }
    }

    private void drive(double now, double dt) {
        double goal = goalAngle();
        inRange = TurretController.isWithinLimits(goal);
        setpoint = inRange ? goal : 0;

        double targetRate = state == State.AIM && inRange ? aim.getTargetRate() : 0;
        double output = controller.calculate(angle, setpoint, targetRate, dt);

        if (controller.isStalled(output, setpoint - angle, velocity, now)) {
            changeState(State.STALL);
            setPower(0);
            return;
        }
        setPower(output);

        if (recovering && state == State.IDLE && isSettled()) recovering = false;
    }

    private void holdStill() {
        controller.reset();
        setpoint = angle;
        setPower(0);
    }

    private void setPower(double output) {
        power = output;
        motor.setPower(output);
    }

    public static double ticksToAngle(double ticks) {
        return ticks / MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO * 2 * Math.PI;
    }

    public boolean isSettled() {
        return !encoderFault
                && Math.abs(setpoint - angle) < Math.toRadians(MOTOR_AIM_TOLERANCE_DEG)
                && Math.abs(velocity) < Math.toRadians(MOTOR_SETTLED_VEL_DEG_S);
    }

    public boolean isAimed() {
        return state == State.AIM && inRange && isSettled() && aim.hasGoodShot();
    }

    public State getState() {
        return state;
    }

    public State getRequestedState() {
        return requestedState;
    }

    public double getAngle() {
        return angle;
    }

    public double getTicks() {
        return ticks;
    }

    public boolean isEncoderFault() {
        return encoderFault;
    }

    public HiveCells.Cell getCell() {
        return aim.getCell();
    }

    public double getTarget() {
        return aim.getTarget();
    }

    public double getDistance() {
        return aim.getDistance();
    }

    public double getBias() {
        return aim.getBias();
    }

    public boolean isSeen() {
        return aim.isSeen();
    }

    public void addTelemetry(TelemetryData telemetry) {
        telemetry.addData("Turret state", recovering ? state + " (recovering)" : state);
        telemetry.addData("Turret requested state", requestedState);
        telemetry.addData("Turret aimed", isAimed());
        telemetry.addData("Robot speed in/s", aim.getRobotSpeed());
        telemetry.addData("Lead time s", aim.getLeadTime());
        telemetry.addData("Turret cell", aim.hasCell() ? aim.getCell() : "none");
        telemetry.addData("Turret tags seen", aim.isSeen());
        telemetry.addData("Turret in range", inRange);
        telemetry.addData("Turret settled", isSettled());
        telemetry.addData("Turret distance in", aim.getDistance());
        telemetry.addData("Turret aim offset deg", AIM_OFFSET_DEG);
        telemetry.addData("Turret target deg", Math.toDegrees(aim.getTarget()));
        telemetry.addData("Turret angle deg", Math.toDegrees(angle));
        telemetry.addData("Turret velocity deg/s", Math.toDegrees(velocity));
        telemetry.addData("Turret error deg", Math.toDegrees(setpoint - angle));
        telemetry.addData("Turret target rate deg/s", Math.toDegrees(aim.getTargetRate()));
        telemetry.addData("Turret feedforward", controller.getFeedforward());
        telemetry.addData("Turret integral power", controller.getIntegralPower());
        telemetry.addData("Turret power", power);
        telemetry.addData("Turret encoder ticks", ticks);
        telemetry.addData("Turret encoder fault", encoderFault);
    }
}
