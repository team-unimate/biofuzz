package org.firstinspires.ftc.teamcode.subsystems.turretmotor;

import static org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMotorConstants.*;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class TurretControllerTest {
    private static final double EPS = 0.0001;
    private static final double FAR = Math.toRadians(20);

    @Test
    public void onTargetGivesNoPower() {
        assertEquals(0, new TurretController().calculate(0.5, 0.5, 0, 0.02), EPS);
    }

    @Test
    public void bigErrorIsClippedToMaxPower() {
        assertEquals(MOTOR_MAX_POWER, new TurretController().calculate(0, Math.toRadians(90), 0, 0.02), EPS);
        assertEquals(-MOTOR_MAX_POWER, new TurretController().calculate(0, Math.toRadians(-90), 0, 0.02), EPS);
    }

    @Test
    public void neverPushesPastTheHardStops() {
        double max = TurretController.maxAngle();
        double min = TurretController.minAngle();
        assertEquals(0, new TurretController().calculate(max, max + FAR, 0, 0.02), EPS);
        assertEquals(0, new TurretController().calculate(min, min - FAR, 0, 0.02), EPS);
        assertTrue(new TurretController().calculate(max, max - FAR, 0, 0.02) < 0);
    }

    @Test
    public void integralIsCappedAndClearedOutsideItsZone() {
        TurretController controller = new TurretController();
        double error = Math.toRadians(MOTOR_I_ZONE_DEG / 2);
        for (int i = 0; i < 100; i++) controller.calculate(0, error, 0, 0.1);
        assertEquals(MOTOR_I_MAX_POWER, controller.getIntegralPower(), EPS);

        controller.calculate(0, Math.toRadians(MOTOR_I_ZONE_DEG * 2), 0, 0.1);
        assertEquals(0, controller.getIntegralPower(), EPS);
    }

    @Test
    public void stallNeedsSustainedPushing() {
        TurretController controller = new TurretController();
        assertFalse(controller.isStalled(0.8, FAR, 0, 0));
        assertFalse(controller.isStalled(0.8, FAR, 0, MOTOR_STALL_TIME_S / 2));
        assertTrue(controller.isStalled(0.8, FAR, 0, MOTOR_STALL_TIME_S));
    }

    @Test
    public void movingRestartsTheStallTimer() {
        TurretController controller = new TurretController();
        controller.isStalled(0.8, FAR, 0, 0);
        controller.isStalled(0.8, FAR, Math.toRadians(90), MOTOR_STALL_TIME_S / 2);
        assertFalse(controller.isStalled(0.8, FAR, 0, MOTOR_STALL_TIME_S));
    }

    @Test
    public void resetClearsTheStallTimer() {
        TurretController controller = new TurretController();
        controller.isStalled(0.8, FAR, 0, 0);
        controller.reset();
        assertFalse(controller.isStalled(0.8, FAR, 0, MOTOR_STALL_TIME_S));
    }
}
