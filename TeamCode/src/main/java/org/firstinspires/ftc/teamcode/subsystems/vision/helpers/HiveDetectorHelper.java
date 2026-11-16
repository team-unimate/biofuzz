package org.firstinspires.ftc.teamcode.subsystems.vision.helpers;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.HiveCells;

import java.util.Collection;
import java.util.function.Supplier;

public class HiveDetectorHelper {
    private final Supplier<Field.Alliance> allianceProvider;

    public HiveDetectorHelper(Supplier<Field.Alliance> allianceProvider) {
        this.allianceProvider = allianceProvider;
    }

    public boolean isTipped(Collection<Integer> visibleTagIds) {
        if (allianceProvider == null) return false;
        Field.Alliance alliance = allianceProvider.get();
        if (alliance == null || visibleTagIds == null) return false;

        int visibleAllianceTags = 0;
        for (int tagId : visibleTagIds) {
            HiveCells.Cell cell = HiveCells.forTag(tagId);
            if (cell != null && cell.alliance == alliance) visibleAllianceTags++;
        }
        return visibleAllianceTags <= 1;
    }
}
