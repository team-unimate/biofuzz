package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.math.Pose;

public final class HiveCells {
    public static final double[] TAG_OFFSETS_IN = {-6.5, -2.75, 2.75, 6.5};

    public enum Cell {
        TOP(38, Field.CELL_TOP),
        BOTTOM(42, Field.CELL_BOTTOM);

        public final int firstTagId;
        public final Pose pose;

        Cell(int firstTagId, Pose pose) {
            this.firstTagId = firstTagId;
            this.pose = pose;
        }

        public int tagIndex(int tagId) {
            int index = tagId - firstTagId;
            return index >= 0 && index < TAG_OFFSETS_IN.length ? index : -1;
        }
    }

    public static Cell forTag(int tagId) {
        for (Cell cell : Cell.values()) {
            if (cell.tagIndex(tagId) >= 0) return cell;
        }
        return null;
    }

    public static Cell forRobot(Pose robotPose) {
        return robotPose.y() >= Field.CENTER_Y ? Cell.TOP : Cell.BOTTOM;
    }

    private HiveCells() {
    }
}
