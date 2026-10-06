package org.firstinspires.ftc.teamcode.subsystems.turret;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class AimMathTest {
    private static final double EPS = 0.000001;

    @Test
    public void statusReportsFirstFailureInOrder() {
        assertEquals(AimMath.Status.CELL_UNKNOWN, AimMath.status(false, true, 5, 24, 120, false, false, false, false, false));
        assertEquals(AimMath.Status.HIVE_JUST_FLIPPED, AimMath.status(true, true, 5, 24, 120, false, false, false, false, false));
        assertEquals(AimMath.Status.TOO_CLOSE, AimMath.status(true, false, 5, 24, 120, false, false, false, false, false));
        assertEquals(AimMath.Status.TOO_CLOSE, AimMath.status(true, false, Double.NaN, 24, 120, true, true, true, true, false));
        assertEquals(AimMath.Status.TOO_FAR, AimMath.status(true, false, 200, 24, 120, false, false, false, false, false));
        assertEquals(AimMath.Status.OUT_OF_RANGE, AimMath.status(true, false, 60, 24, 120, false, false, false, false, false));
        assertEquals(AimMath.Status.NOT_SETTLED, AimMath.status(true, false, 60, 24, 120, true, false, false, false, false));
        assertEquals(AimMath.Status.SHOOTER_NOT_READY, AimMath.status(true, false, 60, 24, 120, true, true, false, false, false));
        assertEquals(AimMath.Status.NO_RECENT_VISION, AimMath.status(true, false, 60, 24, 120, true, true, true, false, false));
        assertEquals(AimMath.Status.OK, AimMath.status(true, false, 60, 24, 120, true, true, true, false, true));
        assertEquals(AimMath.Status.OK, AimMath.status(true, false, 60, 24, 120, true, true, true, true, false));
    }
}
