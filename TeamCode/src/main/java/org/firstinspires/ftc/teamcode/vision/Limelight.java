package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.LimelightConstants.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.Collections;
import java.util.List;

public class Limelight extends SubsystemBase {
    private final Limelight3A limelight;

    private List<LLResultTypes.FiducialResult> visibleTags = Collections.emptyList();
    private boolean resultValid = false;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, DEVICE_NAME);
        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.setPollRateHz(POLL_RATE_HZ);
        limelight.start();
    }

    @Override
    public void periodic() {
        if (!limelight.isConnected()) {
            resultValid = false;
            visibleTags = Collections.emptyList();
            return;
        }

        LLResult result = limelight.getLatestResult();
        resultValid = result != null && result.isValid();
        visibleTags = resultValid ? result.getFiducialResults() : Collections.emptyList();
    }

    public boolean isConnected() {
        return limelight.isConnected();
    }

    public boolean hasValidResult() {
        return resultValid;
    }

    public boolean isTagVisible(int id) {
        return getFiducial(id) != null;
    }

    public LLResultTypes.FiducialResult getFiducial(int id) {
        for (LLResultTypes.FiducialResult tag : visibleTags) {
            if (tag.getFiducialId() == id) return tag;
        }
        return null;
    }

    /** First currently-visible tag whose ID is in {@code ids}, or null if none of them are visible. */
    public LLResultTypes.FiducialResult getFirstVisible(int[] ids) {
        for (LLResultTypes.FiducialResult tag : visibleTags) {
            for (int id : ids) {
                if (tag.getFiducialId() == id) return tag;
            }
        }
        return null;
    }

    public List<LLResultTypes.FiducialResult> getVisibleTags() {
        return visibleTags;
    }
}
