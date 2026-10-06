package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.HiveCells;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretHistory;

import java.util.List;

public final class CellLocator {
    public static class TagPoint {
        public final int id;

        public final int index;
        public final double forward;
        public final double left;
        public final double up;

        public TagPoint(int id, int index, double forward, double left, double up) {
            this.id = id;
            this.index = index;
            this.forward = forward;
            this.left = left;
            this.up = up;
        }
    }

    public static class CameraMount {
        public final double forward;
        public final double left;
        public final double up;
        public final double pitch;
        public final double yaw;

        public CameraMount(double forward, double left, double up, double pitch, double yaw) {
            this.forward = forward;
            this.left = left;
            this.up = up;
            this.pitch = pitch;
            this.yaw = yaw;
        }
    }

    public static class StateGates {
        public final double matchMax;

        public final double tieMargin;

        public final double yawMargin;

        public StateGates(double matchMax, double tieMargin, double yawMargin) {
            this.matchMax = matchMax;
            this.tieMargin = tieMargin;
            this.yawMargin = yawMargin;
        }
    }

    public static double[] cameraToTurret(double xRight, double yDown, double zForward, CameraMount mount) {
        double cp = Math.cos(mount.pitch), sp = Math.sin(mount.pitch);
        double cy = Math.cos(mount.yaw), sy = Math.sin(mount.yaw);

        double[] fwd = {cp * cy, cp * sy, sp};
        double[] right = {sy, -cy, 0};
        double[] down = {sp * cy, sp * sy, -cp};
        return new double[]{
                mount.forward + xRight * right[0] + yDown * down[0] + zForward * fwd[0],
                mount.left + xRight * right[1] + yDown * down[1] + zForward * fwd[1],
                mount.up + xRight * right[2] + yDown * down[2] + zForward * fwd[2]
        };
    }

    public static double measureRowYaw(List<TagPoint> tags) {
        double sumF = 0, sumL = 0;
        for (int i = 0; i < tags.size(); i++) {
            for (int j = i + 1; j < tags.size(); j++) {
                TagPoint a = tags.get(i), b = tags.get(j);
                double span = HiveCells.TAG_OFFSETS_IN[b.index] - HiveCells.TAG_OFFSETS_IN[a.index];
                if (span == 0) continue;

                sumF += (b.forward - a.forward) * Math.signum(span);
                sumL += (b.left - a.left) * Math.signum(span);
            }
        }
        return sumF == 0 && sumL == 0 ? Double.NaN : Math.atan2(sumL, sumF);
    }

    public static double[] rowCenter(List<TagPoint> tags, double rowYaw) {
        double cos = Math.cos(rowYaw), sin = Math.sin(rowYaw);
        double f = 0, l = 0;
        for (TagPoint tag : tags) {
            double offset = HiveCells.TAG_OFFSETS_IN[tag.index];
            f += tag.forward - offset * cos;
            l += tag.left - offset * sin;
        }
        return new double[]{f / tags.size(), l / tags.size()};
    }

    public static double[] turretToField(double forward, double left, TurretHistory.Sample at,
                                         double pivotForward, double pivotLeft) {
        double ch = Math.cos(at.heading), sh = Math.sin(at.heading);
        double pivotX = at.x + pivotForward * ch - pivotLeft * sh;
        double pivotY = at.y + pivotForward * sh + pivotLeft * ch;
        double a = at.heading + at.turretAngle;
        return new double[]{
                pivotX + forward * Math.cos(a) - left * Math.sin(a),
                pivotY + forward * Math.sin(a) + left * Math.cos(a)
        };
    }

    public static CellObservation locate(HiveCells.Cell cell, List<TagPoint> tags, TurretHistory.Sample at,
                                         double pivotForward, double pivotLeft, StateGates gates) {
        double turretToFieldYaw = at.heading + at.turretAngle;
        double measuredYaw = measureRowYaw(tags);

        double[] center;
        HiveCells.State state;
        if (!Double.isNaN(measuredYaw)) {
            center = rowCenter(tags, measuredYaw);
            double[] field = turretToField(center[0], center[1], at, pivotForward, pivotLeft);
            HiveCells.State byPosition = closest(
                    distance(field, cell.pose(HiveCells.State.UP)),
                    distance(field, cell.pose(HiveCells.State.DOWN)),
                    gates.matchMax, gates.tieMargin);
            double fieldYaw = measuredYaw + turretToFieldYaw;
            HiveCells.State byYaw = closest(
                    Math.abs(wrap(fieldYaw - cell.pose(HiveCells.State.UP).heading())),
                    Math.abs(wrap(fieldYaw - cell.pose(HiveCells.State.DOWN).heading())),
                    Double.POSITIVE_INFINITY, gates.yawMargin);

            state = byYaw == HiveCells.State.UNKNOWN || byYaw == byPosition ? byPosition : HiveCells.State.UNKNOWN;
        } else {
            double[] upCenter = rowCenter(tags, wrap(cell.pose(HiveCells.State.UP).heading() - turretToFieldYaw));
            double[] downCenter = rowCenter(tags, wrap(cell.pose(HiveCells.State.DOWN).heading() - turretToFieldYaw));
            state = closest(
                    distance(turretToField(upCenter[0], upCenter[1], at, pivotForward, pivotLeft), cell.pose(HiveCells.State.UP)),
                    distance(turretToField(downCenter[0], downCenter[1], at, pivotForward, pivotLeft), cell.pose(HiveCells.State.DOWN)),
                    gates.matchMax, gates.tieMargin);

            center = state == HiveCells.State.DOWN ? downCenter : upCenter;
        }

        double[] field = turretToField(center[0], center[1], at, pivotForward, pivotLeft);
        int[] ids = new int[tags.size()];
        for (int i = 0; i < ids.length; i++) ids[i] = tags.get(i).id;
        return new CellObservation(cell, ids, center[0], center[1], measuredYaw, state,
                field[0], field[1], at.seconds, at.turretAngle, at.heading);
    }

    static HiveCells.State closest(double upError, double downError, double max, double margin) {
        if (Math.abs(upError - downError) <= margin) return HiveCells.State.UNKNOWN;
        if (upError < downError) return upError <= max ? HiveCells.State.UP : HiveCells.State.UNKNOWN;
        return downError <= max ? HiveCells.State.DOWN : HiveCells.State.UNKNOWN;
    }

    private static double distance(double[] field, Pose pose) {
        return Math.hypot(field[0] - pose.x(), field[1] - pose.y());
    }

    static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private CellLocator() {
    }
}
