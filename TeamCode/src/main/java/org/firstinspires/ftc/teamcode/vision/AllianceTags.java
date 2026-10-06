package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.field.Field;

public class AllianceTags {
    public static int[] RED_TAG_IDS = {30, 31, 32, 33, 34, 35, 36, 37};
    public static int[] BLUE_TAG_IDS = {38, 39, 40, 41, 42, 43, 44, 45};

    public static int[] tagIdsFor(Field.Alliance alliance) {
        return alliance == Field.Alliance.RED ? RED_TAG_IDS : BLUE_TAG_IDS;
    }
}
