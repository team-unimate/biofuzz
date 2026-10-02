package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.field.Field;

/**
 * Which AprilTag IDs are valid to aim at for each alliance ({@link Field.Alliance}). Several tags
 * can belong to the same alliance (e.g. visible from different angles/structures) - the aimer
 * tracks whichever one of them is currently visible.
 * <p>
 * These IDs are confirmed for this game (BioBuzz): they match both the field diagram (RED SCORING
 * 30-33 / RED AUDIENCE 34-37 / BLUE AUDIENCE 38-41 / BLUE SCORING 42-45) and the FTC SDK's own
 * {@code AprilTagGameDatabase.getBioBuzzTagLibrary()} cluster groupings.
 * <p>
 * Not a dashboard-tunable constants class (arrays don't edit well there); change these in code.
 */
public class AllianceTags {
    public static int[] RED_TAG_IDS = {30, 31, 32, 33, 34, 35, 36, 37};
    public static int[] BLUE_TAG_IDS = {38, 39, 40, 41, 42, 43, 44, 45};

    public static int[] tagIdsFor(Field.Alliance alliance) {
        return alliance == Field.Alliance.RED ? RED_TAG_IDS : BLUE_TAG_IDS;
    }
}
