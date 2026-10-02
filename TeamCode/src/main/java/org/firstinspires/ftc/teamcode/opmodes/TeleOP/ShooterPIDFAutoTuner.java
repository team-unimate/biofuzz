package org.firstinspires.ftc.teamcode.opmodes.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;

/**
 * Automatically tunes the shooter's PIDF gains in four sequential phases - kF, then kP, kD, kI -
 * each capped at PHASE_BUDGET_MS (1 minute), for a 4 minute total worst case.
 * <p>
 * kF: the flywheel is driven open-loop near its actual operating point (HIVE_RPM, not an
 * arbitrary 0-100% sweep - see characterizeFeedforward()'s doc) and
 * kF is fit as the least-squares slope of power vs. settled velocity, matching how
 * {@code com.seattlesolvers.solverslib.controller.PIDFController} applies it: power += kF *
 * targetVelocity.
 * <p>
 * kP/kD/kI: each is tuned alone, in that order (the standard manual sequence - P first for
 * responsiveness, D to damp the overshoot P introduces, I last to trim any remaining
 * steady-state error), with a single-parameter search over a multiplicative step (try the value
 * scaled up or down by a percentage, not by a fixed absolute amount) - see tuneSingleGain()'s doc
 * for why. Every trial is a closed-loop step response to the test RPM, scored by ITAE
 * (time-weighted error) plus an overshoot penalty.
 * <p>
 * Results are written live into ShooterConstants (visible on the FTC Dashboard as they change),
 * but that's a runtime-only edit - copy the final printed values back into ShooterConstants.java
 * once the run finishes.
 */
@TeleOp(name = "Shooter PIDF AutoTuner", group = "Tuning")
public class ShooterPIDFAutoTuner extends LinearOpMode {
    private static final double PHASE_BUDGET_MS = 60_000;

    private static final double FF_BRACKET_STEP_MS = 400;
    private static final double FF_BRACKET_POWER_STEP = 0.05;
    private static final double FF_SAMPLE_SETTLE_MS = 700;
    private static final double FF_SAMPLE_WINDOW_MS = 300;

    private static final double TRIAL_SETTLE_MAX_MS = 800;
    private static final double STOP_VELOCITY_TOLERANCE = 50;
    private static final double TRIAL_DURATION_MS = 900;
    private static final double OVERSHOOT_PENALTY = 0.5;

    // Gain search step is multiplicative (a fraction of the current value), not a fixed
    // absolute amount: kP/kD/kI can be unknown by orders of magnitude (e.g. kP ~1e-2 vs.
    // kD ~1e-5), and an additive step sized for one is either useless or wildly unstable for
    // another. A multiplicative step also lets the search climb from a near-zero seed to a
    // realistic gain in a handful of trials (doubling 10 times covers 1024x) instead of
    // needing dozens of tiny +10% increments that can't fit in PHASE_BUDGET_MS.
    private static final double GAIN_ZERO_SEED = 1e-5; // can't scale away from a literal 0
    private static final double STEP_FACTOR_INITIAL = 1.0; // first trial doubles/halves the value
    private static final double STEP_FACTOR_MAX = 3.0;
    private static final double STEP_FACTOR_MIN = 0.02; // stop once candidates are within ~2%
    private static final double STEP_FACTOR_GROWTH = 1.3;
    private static final double STEP_FACTOR_SHRINK = 0.6;

    private interface GainSetter {
        void set(double value);
    }

    private Shooter shooter;
    private double testRPM;

    @Override
    public void runOpMode() {
        shooter = new Shooter(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        testRPM = ShooterConstants.HIVE_RPM;

        telemetry.addLine("Shooter PIDF AutoTuner ready.");
        telemetry.addLine("Flywheel will spin during this routine - clear the area in front of it.");
        telemetry.addLine("4 phases, 1 minute each: kF, kP, kD, kI.");
        telemetry.addData("Test RPM", testRPM);
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        try {
            ShooterConstants.kF = characterizeFeedforward();

            double seedKD = ShooterConstants.kD;
            double seedKI = ShooterConstants.kI;
            ShooterConstants.kI = 0;
            ShooterConstants.kD = 0;

            ShooterConstants.kP = tuneSingleGain("kP", ShooterConstants.kP, v -> ShooterConstants.kP = v);
            ShooterConstants.kD = tuneSingleGain("kD", seedKD, v -> ShooterConstants.kD = v);
            ShooterConstants.kI = tuneSingleGain("kI", seedKI, v -> ShooterConstants.kI = v);
        } finally {
            // Guarantees the flywheel is commanded to stop even if the routine is interrupted
            // (driver station stop) partway through.
            shooter.clearOpenLoopOverride();
            shooter.stop();
            shooter.periodic();
        }

        while (opModeIsActive()) {
            shooter.periodic();
            telemetry.addLine("DONE - copy these into ShooterConstants.java:");
            telemetry.addData("kF", format(ShooterConstants.kF));
            telemetry.addData("kP", format(ShooterConstants.kP));
            telemetry.addData("kI", format(ShooterConstants.kI));
            telemetry.addData("kD", format(ShooterConstants.kD));
            telemetry.update();
        }
    }

    /**
     * Characterizes kF near the actual operating point instead of sweeping an arbitrary 0-100%
     * power range. The flywheel's real power/velocity relationship has a static-friction
     * intercept (power = kStatic + kV*velocity), but PIDFController only supports a pure
     * proportional-to-setpoint kF term with no intercept - fitting that single-parameter model
     * against samples taken well above the actual target RPM bakes the high-speed
     * friction/back-EMF behavior into the slope, which then overshoots badly when applied at the
     * much lower real target. So: first bracket the power that roughly reaches the test RPM, then
     * sample only in a narrow band around it.
     */
    private double characterizeFeedforward() {
        double targetVelocity = testRPM * ShooterConstants.TICKS_PER_REV / 60.0;
        ElapsedTime phaseTimer = new ElapsedTime();

        double bracketPower = 0;
        double bracketVelocity = 0;
        while (opModeIsActive()
                && phaseTimer.milliseconds() < PHASE_BUDGET_MS * 0.4
                && bracketPower < 1.0
                && bracketVelocity < targetVelocity) {
            bracketPower = Math.min(1.0, bracketPower + FF_BRACKET_POWER_STEP);
            shooter.setOpenLoopPower(bracketPower);
            runFor(FF_BRACKET_STEP_MS);
            bracketVelocity = shooter.getVelocity();

            telemetry.addLine("Characterizing kF: bracketing the operating power...");
            telemetry.addData("Power", bracketPower);
            telemetry.addData("Velocity (ticks/s)", bracketVelocity);
            telemetry.addData("Target velocity (ticks/s)", targetVelocity);
            telemetry.update();
        }

        double[] samplePowers = {
                Math.max(0.05, bracketPower * 0.7),
                bracketPower,
                Math.min(1.0, bracketPower * 1.3)
        };

        double sumPowerVelocity = 0;
        double sumVelocitySquared = 0;

        for (double power : samplePowers) {
            if (!opModeIsActive() || phaseTimer.milliseconds() > PHASE_BUDGET_MS) break;

            shooter.setOpenLoopPower(power);
            runFor(FF_SAMPLE_SETTLE_MS);
            if (!opModeIsActive()) break;

            double velocitySum = 0;
            int samples = 0;
            ElapsedTime sampleTimer = new ElapsedTime();
            while (opModeIsActive() && sampleTimer.milliseconds() < FF_SAMPLE_WINDOW_MS) {
                shooter.periodic();
                velocitySum += shooter.getVelocity();
                samples++;

                telemetry.addLine("Characterizing kF: sampling near the operating point...");
                telemetry.addData("Power", power);
                telemetry.addData("Velocity (ticks/s)", shooter.getVelocity());
                telemetry.update();
            }

            if (samples > 0) {
                double avgVelocity = velocitySum / samples;
                sumPowerVelocity += power * avgVelocity;
                sumVelocitySquared += avgVelocity * avgVelocity;
            }
        }

        shooter.setOpenLoopPower(0);
        runFor(300);
        shooter.clearOpenLoopOverride();

        return sumVelocitySquared > 1e-6 ? sumPowerVelocity / sumVelocitySquared : ShooterConstants.kF;
    }

    /**
     * Single-parameter search over a multiplicative step, capped at PHASE_BUDGET_MS - the other
     * two gains are held fixed by the caller. Each trial tries value*(1+stepFactor) and, if that's
     * not better, value/(1+stepFactor); whichever improves the cost is kept and stepFactor grows
     * for the next trial, otherwise stepFactor shrinks to refine around the current best.
     */
    private double tuneSingleGain(String label, double initialValue, GainSetter setter) {
        double value = initialValue > 0 ? initialValue : GAIN_ZERO_SEED;
        setter.set(value);
        double stepFactor = STEP_FACTOR_INITIAL;
        double bestCost = trialCost();

        ElapsedTime phaseTimer = new ElapsedTime();
        while (opModeIsActive() && phaseTimer.milliseconds() < PHASE_BUDGET_MS && stepFactor > STEP_FACTOR_MIN) {
            double original = value;

            double up = original * (1 + stepFactor);
            setter.set(up);
            double cost = trialCost();
            if (cost < bestCost) {
                bestCost = cost;
                value = up;
                stepFactor = Math.min(STEP_FACTOR_MAX, stepFactor * STEP_FACTOR_GROWTH);
            } else {
                double down = original / (1 + stepFactor);
                setter.set(down);
                cost = trialCost();
                if (cost < bestCost) {
                    bestCost = cost;
                    value = down;
                    stepFactor = Math.min(STEP_FACTOR_MAX, stepFactor * STEP_FACTOR_GROWTH);
                } else {
                    value = original;
                    setter.set(value);
                    stepFactor *= STEP_FACTOR_SHRINK;
                }
            }

            telemetry.addLine("Tuning " + label + " (search)...");
            telemetry.addData(label, format(value));
            telemetry.addData("Best cost", format(bestCost));
            telemetry.addData("Time left (s)", Math.max(0, PHASE_BUDGET_MS - phaseTimer.milliseconds()) / 1000.0);
            telemetry.update();
        }

        return value;
    }

    /** Fixed-point formatting for telemetry - scientific notation (e.g. "1.0E-6") is hard to read at a glance. */
    private static String format(double value) {
        return String.format("%.8f", value);
    }

    /**
     * Runs one closed-loop step response to testRPM with whatever's currently in ShooterConstants
     * and scores it with ITAE (error weighted by elapsed time) rather than plain integrated error,
     * so the opening spin-up ramp - limited by the motor's physical acceleration, not by the gain
     * being tested - doesn't drown out how well the candidate actually settles/tracks.
     * <p>
     * Returns Double.POSITIVE_INFINITY if the routine is stopped before the trial completes, so a
     * cut-short trial (with artificially little accumulated error) can never be mistaken for a
     * genuinely good result.
     */
    private double trialCost() {
        shooter.stop();
        shooter.resetController();
        if (!waitForNearZeroVelocity(TRIAL_SETTLE_MAX_MS)) {
            return Double.POSITIVE_INFINITY;
        }

        shooter.setTargetRPM(testRPM);
        double weightedError = 0;
        double peakRPM = 0;
        double lastMillis = 0;
        ElapsedTime timer = new ElapsedTime();

        while (timer.milliseconds() < TRIAL_DURATION_MS) {
            if (!opModeIsActive()) return Double.POSITIVE_INFINITY;
            shooter.periodic();

            double nowMillis = timer.milliseconds();
            double dt = nowMillis - lastMillis;
            lastMillis = nowMillis;

            double rpm = shooter.getRPM();
            peakRPM = Math.max(peakRPM, rpm);
            weightedError += Math.abs(testRPM - rpm) * dt * (nowMillis / 1000.0);
        }

        double overshoot = Math.max(0, peakRPM - testRPM);
        return weightedError + OVERSHOOT_PENALTY * overshoot * TRIAL_DURATION_MS;
    }

    /** Waits for the flywheel to coast down near zero so each trial starts from the same state. */
    private boolean waitForNearZeroVelocity(double maxWaitMs) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < maxWaitMs) {
            shooter.periodic();
            if (Math.abs(shooter.getVelocity()) < STOP_VELOCITY_TOLERANCE) {
                return true;
            }
        }
        return opModeIsActive();
    }

    private void runFor(double ms) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < ms) {
            shooter.periodic();
        }
    }
}
