package org.firstinspires.ftc.teamcode.subsystems.vision.helpers;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.field.Field;
import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

public class HiveDetectorHelperTest {
    private final AtomicReference<Field.Alliance> alliance = new AtomicReference<>(Field.Alliance.RED);
    private final Supplier<Field.Alliance> allianceProvider = alliance::get;
    private final HiveDetectorHelper helper = new HiveDetectorHelper(allianceProvider);

    @Test
    public void allianceTagCountDeterminesTippedState() {
        assertFalse(helper.isTipped(Arrays.asList(30, 34)));
        assertTrue(helper.isTipped(Arrays.asList(30, 38)));

        alliance.set(Field.Alliance.BLUE);
        assertTrue(helper.isTipped(Collections.singletonList(42)));
    }

    @Test
    public void missingAllianceOrTagsDoesNotClaimTipped() {
        alliance.set(null);
        assertFalse(helper.isTipped(Collections.<Integer>emptyList()));

        alliance.set(Field.Alliance.BLUE);
        assertFalse(helper.isTipped(null));
    }
}
