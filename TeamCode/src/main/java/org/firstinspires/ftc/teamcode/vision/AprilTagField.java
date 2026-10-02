package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.field.Field;

/**
 * Known field positions of the AprilTags the robot can aim at, in the same coordinate frame as
 * Pedro Pathing's Follower pose (inches, +x forward, +y left) used elsewhere in this project (see
 * {@link Field}).
 * <p>
 * The FTC SDK's {@code AprilTagGameDatabase.getBioBuzzTagLibrary()} does NOT provide field
 * placement - it only encodes each 4-tag plate's tiny internal layout (a few inches between tags
 * 30/31/32/33 etc.), not where that plate sits on the field; every cluster's own field position in
 * the SDK is (0,0,0) with no rotation, i.e. unset. So these reuse {@link Field}'s existing goal
 * poses instead. Since the 4 tags on one plate are only a few inches apart, one field point per
 * plate is accurate enough for heading-aim purposes - all 4 IDs in a cluster share one entry.
 * <p>
 * Confirmed: BLUE SCORING (42-45) is at {@link Field#BLUE_CELL_TOP}. The other three below follow
 * by the same top=scoring/bottom=audience pattern for the other color - NOT independently
 * confirmed, verify before relying on them. {@link Field}'s own poses are themselves still marked
 * as placeholders pending measurement, so these inherit that uncertainty too.
 */
public class AprilTagField {
    public static final AprilTagTarget[] TARGETS = {
            // BLUE SCORING - confirmed at BLUE_CELL_TOP.
            new AprilTagTarget(42, Field.BLUE_CELL_TOP.x(), Field.BLUE_CELL_TOP.y()),
            new AprilTagTarget(43, Field.BLUE_CELL_TOP.x(), Field.BLUE_CELL_TOP.y()),
            new AprilTagTarget(44, Field.BLUE_CELL_TOP.x(), Field.BLUE_CELL_TOP.y()),
            new AprilTagTarget(45, Field.BLUE_CELL_TOP.x(), Field.BLUE_CELL_TOP.y()),

            // BLUE AUDIENCE - inferred (bottom = audience), verify.
            new AprilTagTarget(38, Field.BLUE_CELL_BOTTOM.x(), Field.BLUE_CELL_BOTTOM.y()),
            new AprilTagTarget(39, Field.BLUE_CELL_BOTTOM.x(), Field.BLUE_CELL_BOTTOM.y()),
            new AprilTagTarget(40, Field.BLUE_CELL_BOTTOM.x(), Field.BLUE_CELL_BOTTOM.y()),
            new AprilTagTarget(41, Field.BLUE_CELL_BOTTOM.x(), Field.BLUE_CELL_BOTTOM.y()),

            // RED SCORING - inferred (top = scoring, mirroring BLUE), verify.
            new AprilTagTarget(30, Field.RED_CELL_TOP.x(), Field.RED_CELL_TOP.y()),
            new AprilTagTarget(31, Field.RED_CELL_TOP.x(), Field.RED_CELL_TOP.y()),
            new AprilTagTarget(32, Field.RED_CELL_TOP.x(), Field.RED_CELL_TOP.y()),
            new AprilTagTarget(33, Field.RED_CELL_TOP.x(), Field.RED_CELL_TOP.y()),

            // RED AUDIENCE - inferred (bottom = audience), verify.
            new AprilTagTarget(34, Field.RED_CELL_BOTTOM.x(), Field.RED_CELL_BOTTOM.y()),
            new AprilTagTarget(35, Field.RED_CELL_BOTTOM.x(), Field.RED_CELL_BOTTOM.y()),
            new AprilTagTarget(36, Field.RED_CELL_BOTTOM.x(), Field.RED_CELL_BOTTOM.y()),
            new AprilTagTarget(37, Field.RED_CELL_BOTTOM.x(), Field.RED_CELL_BOTTOM.y()),
    };

    public static AprilTagTarget get(int id) {
        for (AprilTagTarget target : TARGETS) {
            if (target.id == id) return target;
        }
        return null;
    }
}
