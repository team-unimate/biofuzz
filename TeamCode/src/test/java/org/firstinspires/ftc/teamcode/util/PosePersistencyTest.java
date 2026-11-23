package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.junit.Test;

public class PosePersistencyTest {
    @Test
    public void fallsBackUntilSavedAndClears() {
        PosePersistency.clear();
        Pose fallback = new Pose(1, 2, 3);
        assertFalse(PosePersistency.hasPose());
        assertSame(fallback, PosePersistency.getPose(fallback));
        assertEquals(7, PosePersistency.getTurretMotorZero(7), 0);

        Pose saved = new Pose(10, 20, 0.5);
        PosePersistency.savePose(saved);
        PosePersistency.saveTurretMotorZero(42);
        assertTrue(PosePersistency.hasPose());
        assertSame(saved, PosePersistency.getPose(fallback));
        assertEquals(42, PosePersistency.getTurretMotorZero(7), 0);

        PosePersistency.clear();
        assertSame(fallback, PosePersistency.getPose(fallback));
        assertEquals(7, PosePersistency.getTurretMotorZero(7), 0);
    }
}
