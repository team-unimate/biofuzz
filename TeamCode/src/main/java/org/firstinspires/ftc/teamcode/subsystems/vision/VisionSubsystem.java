package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.subsystems.vision.helpers.HiveDetectorHelper;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class VisionSubsystem extends SubsystemBase {
    //this implementation for the subsystem isnt the only one, this one is meant to use the limelight native sdk class
    private final Limelight3A limelight;
    private final HiveDetectorHelper hiveDetectorHelper;

    public VisionSubsystem(HardwareMap hardwareMap, Supplier<Field.Alliance> allianceProvider) {
        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.LIMELIGHT_HARDWARE_NAME);
        hiveDetectorHelper = new HiveDetectorHelper(allianceProvider);
        limelight.start();
    }

    public boolean isHiveTipped() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return false;

        List<Integer> visibleTagIds = new ArrayList<>();
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            visibleTagIds.add(tag.getFiducialId());
        }
        return hiveDetectorHelper.isTipped(visibleTagIds);
    }

    public Limelight3A getLimelight() {
        return limelight;
    }
}
