package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Shooter extends SubsystemBase {
    private final MotorEx flywheel;
    private final PIDFController controller = new PIDFController(kP, kI, kD, kF);

    private double targetRPM = 0;
    private double power = 0;

    private boolean openLoopOverride = false;
    private double openLoopPower = 0;

    public Shooter(HardwareMap hardwareMap) {
        flywheel = new MotorEx(hardwareMap, MOTOR_NAME);
        flywheel.setRunMode(Motor.RunMode.RawPower);
        flywheel.setInverted(MOTOR_REVERSED);
        flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        controller.setTolerance(VELOCITY_TOLERANCE);
    }

    public void setTargetRPM(double rpm) {
        targetRPM = Range.clip(rpm, 0, MAX_RPM);
    }

    public void stop() {
        targetRPM = 0;
    }

    /** Bypasses the PIDF loop and drives the flywheel at a fixed power - used for feedforward (kF) characterization. */
    public void setOpenLoopPower(double power) {
        openLoopOverride = true;
        openLoopPower = Range.clip(power, -1, 1);
    }

    public void clearOpenLoopOverride() {
        openLoopOverride = false;
        openLoopPower = 0;
    }

    /** Clears accumulated integral/derivative state - call between auto-tuner trials. */
    public void resetController() {
        controller.reset();
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getRPM() {
        return getVelocity() * 60.0 / TICKS_PER_REV;
    }

    public double getVelocity() {
        return flywheel.motorEx.getVelocity();
    }

    public double getPower() {
        return power;
    }

    public boolean atTarget() {
        return targetRPM > 0 && controller.atSetPoint();
    }

    @Override
    public void periodic() {
        flywheel.setInverted(MOTOR_REVERSED);

        if (openLoopOverride) {
            power = openLoopPower;
            flywheel.set(power);
            return;
        }

        // Re-applied every loop so dashboard edits to ShooterConstants take effect immediately.
        controller.setPIDF(kP, kI, kD, kF);
        controller.setTolerance(VELOCITY_TOLERANCE);
        controller.integrationControl.setIntegrationBounds(INTEGRAL_MIN, INTEGRAL_MAX);

        if (targetRPM <= 0) {
            power = 0;
            flywheel.set(0);
            return;
        }

        double targetTicksPerSecond = targetRPM * TICKS_PER_REV / 60.0;
        controller.setSetPoint(targetTicksPerSecond);
        power = Range.clip(controller.calculate(getVelocity()), 0, 1);
        flywheel.set(power);
    }
}
