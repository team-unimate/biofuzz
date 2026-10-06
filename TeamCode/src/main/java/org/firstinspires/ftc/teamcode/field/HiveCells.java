package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.math.Pose;

public final class HiveCells {
    public static final double[] TAG_OFFSETS_IN = {-6.5, -2.75, 2.75, 6.5};

    public enum State {UP, DOWN, UNKNOWN}

    public enum Cell {
        RED_FAR(Field.Alliance.RED, 30, Field.RED_CELL_TOP, Field.RED_CELL_TOP),
        RED_AUDIENCE(Field.Alliance.RED, 34, Field.RED_CELL_BOTTOM, Field.RED_CELL_BOTTOM),
        BLUE_AUDIENCE(Field.Alliance.BLUE, 38, Field.BLUE_CELL_BOTTOM, Field.BLUE_CELL_BOTTOM),
        BLUE_FAR(Field.Alliance.BLUE, 42, Field.BLUE_CELL_TOP, Field.BLUE_CELL_TOP);

        public final Field.Alliance alliance;
        public final int firstTagId;
        private final Pose up;
        private final Pose down;

        Cell(Field.Alliance alliance, int firstTagId, Pose up, Pose down) {
            this.alliance = alliance;
            this.firstTagId = firstTagId;
            this.up = up;
            this.down = down;
        }

        public Pose pose(State state) {
            if (state == State.UNKNOWN) throw new IllegalArgumentException("No pose for UNKNOWN");
            return state == State.UP ? up : down;
        }

        public int tagIndex(int tagId) {
            int index = tagId - firstTagId;
            return index >= 0 && index < TAG_OFFSETS_IN.length ? index : -1;
        }

        public Cell partner() {
            switch (this) {
                case RED_FAR: return RED_AUDIENCE;
                case RED_AUDIENCE: return RED_FAR;
                case BLUE_AUDIENCE: return BLUE_FAR;
                default: return BLUE_AUDIENCE;
            }
        }
    }

    public static Cell forTag(int tagId) {
        for (Cell cell : Cell.values()) {
            if (cell.tagIndex(tagId) >= 0) return cell;
        }
        return null;
    }

    private HiveCells() {
    }
}
