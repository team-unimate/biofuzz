package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.HiveCells;

public final class PosePersistency {
    private static Pose pose = null;
    private static Field.Alliance alliance = null;
    private static HiveCells.Cell upCell = null;

    public static void savePose(Pose pose) {
        PosePersistency.pose = pose;
    }

    public static void saveAlliance(Field.Alliance alliance) {
        PosePersistency.alliance = alliance;
    }

    public static void saveUpCell(HiveCells.Cell upCell) {
        PosePersistency.upCell = upCell;
    }

    public static Pose getPose(Pose fallback) {
        return pose != null ? pose : fallback;
    }

    public static Field.Alliance getAlliance(Field.Alliance fallback) {
        return alliance != null ? alliance : fallback;
    }

    public static HiveCells.Cell getUpCell() {
        return upCell;
    }

    public static boolean hasPose() {
        return pose != null;
    }

    public static void clear() {
        pose = null;
        alliance = null;
        upCell = null;
    }

    private PosePersistency() {
    }
}
