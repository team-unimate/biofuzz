package org.firstinspires.ftc.teamcode.subsystems.turretmotor;

import static org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMotorConstants.*;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;

public class TurretController {
    private final PIDController pid = new PIDController(MOTOR_kP, 0, MOTOR_kD);
    private double integral = 0;
    private double feedforward = 0;
    private double stallStart = Double.NaN;

    public void reset() {
        pid.reset();
        integral = 0;
        stallStart = Double.NaN;
    }

    public double calculate(double angle, double setpoint, double targetRate, double dt) {
        double error = setpoint - angle;
        pid.setPID(MOTOR_kP, 0, MOTOR_kD);
        double output = pid.calculate(angle, setpoint);
        output += integralPower(error, dt);

        feedforward = MOTOR_kV * targetRate;
        output += feedforward;

        if (Math.abs(error) > Math.toRadians(MOTOR_DEADBAND_DEG)) output += MOTOR_kS * Math.signum(error);
        output = Range.clip(output, -MOTOR_MAX_POWER, MOTOR_MAX_POWER);

        if ((angle >= maxAngle() && output > 0) || (angle <= minAngle() && output < 0)) output = 0;
        return output;
    }

    private double integralPower(double error, double dt) {
        if (Math.abs(error) > Math.toRadians(MOTOR_I_ZONE_DEG)) {
            integral = 0;
        } else if (Math.abs(error) > Math.toRadians(MOTOR_DEADBAND_DEG) && MOTOR_kI > 0) {
            double limit = MOTOR_I_MAX_POWER / MOTOR_kI;
            integral = Range.clip(integral + error * dt, -limit, limit);
        }
        return MOTOR_kI * integral;
    }

    public boolean isStalled(double output, double error, double velocity, double now) {
        if (!isPushingWithoutMoving(output, error, velocity)) {
            stallStart = Double.NaN;
            return false;
        }
        if (Double.isNaN(stallStart)) {
            stallStart = now;
            return false;
        }
        return now - stallStart >= MOTOR_STALL_TIME_S;
    }

    static boolean isPushingWithoutMoving(double output, double error, double velocity) {
        return Math.abs(output) >= MOTOR_STALL_MIN_POWER
                && Math.abs(error) > Math.toRadians(MOTOR_AIM_TOLERANCE_DEG)
                && Math.abs(velocity) < Math.toRadians(MOTOR_STALL_VEL_DEG_S);
    }

    public static double minAngle() {
        return Math.toRadians(MOTOR_MIN_ANGLE_DEG);
    }

    public static double maxAngle() {
        return Math.toRadians(MOTOR_MAX_ANGLE_DEG);
    }

    public static boolean isWithinLimits(double angle) {
        return angle >= minAngle() && angle <= maxAngle();
    }

    public double getFeedforward() {
        return feedforward;
    }

    public double getIntegralPower() {
        return MOTOR_kI * integral;
    }
}
