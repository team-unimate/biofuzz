package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.field.Field;

public class AprilTagField {
    public static final AprilTagTarget[] TARGETS = {
            // BLUE SCORING
            new AprilTagTarget(42, Field.BLUE_CELL_TOP.x(), Field.BLUE_CELL_TOP.y()),
            new AprilTagTarget(43, Field.BLUE_CELL_TOP.x(), Field.BLUE_CELL_TOP.y()),
            new AprilTagTarget(44, Field.BLUE_CELL_TOP.x(), Field.BLUE_CELL_TOP.y()),
            new AprilTagTarget(45, Field.BLUE_CELL_TOP.x(), Field.BLUE_CELL_TOP.y()),

            // BLUE AUDIENCE
            new AprilTagTarget(38, Field.BLUE_CELL_BOTTOM.x(), Field.BLUE_CELL_BOTTOM.y()),
            new AprilTagTarget(39, Field.BLUE_CELL_BOTTOM.x(), Field.BLUE_CELL_BOTTOM.y()),
            new AprilTagTarget(40, Field.BLUE_CELL_BOTTOM.x(), Field.BLUE_CELL_BOTTOM.y()),
            new AprilTagTarget(41, Field.BLUE_CELL_BOTTOM.x(), Field.BLUE_CELL_BOTTOM.y()),

            // RED SCORING
            new AprilTagTarget(30, Field.RED_CELL_TOP.x(), Field.RED_CELL_TOP.y()),
            new AprilTagTarget(31, Field.RED_CELL_TOP.x(), Field.RED_CELL_TOP.y()),
            new AprilTagTarget(32, Field.RED_CELL_TOP.x(), Field.RED_CELL_TOP.y()),
            new AprilTagTarget(33, Field.RED_CELL_TOP.x(), Field.RED_CELL_TOP.y()),

            // RED AUDIENCE
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
