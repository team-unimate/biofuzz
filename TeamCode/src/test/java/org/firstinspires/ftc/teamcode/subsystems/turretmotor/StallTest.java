package org.firstinspires.ftc.teamcode.subsystems.turretmotor;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class StallTest {
    private static final double FAR = Math.toRadians(20);
    private static final double CLOSE = Math.toRadians(0.5);
    private static final double STOPPED = 0;
    private static final double MOVING = Math.toRadians(90);

    @Test
    public void pushingHardAndNotMovingIsStall() {
        assertTrue(TurretController.isPushingWithoutMoving(0.8, FAR, STOPPED));
        assertTrue(TurretController.isPushingWithoutMoving(-0.8, -FAR, STOPPED));
    }

    @Test
    public void movingIsNotStall() {
        assertFalse(TurretController.isPushingWithoutMoving(0.8, FAR, MOVING));
    }

    @Test
    public void lowPowerIsNotStall() {
        assertFalse(TurretController.isPushingWithoutMoving(0.1, FAR, STOPPED));
    }

    @Test
    public void onTargetIsNotStall() {
        assertFalse(TurretController.isPushingWithoutMoving(0.8, CLOSE, STOPPED));
    }
}
