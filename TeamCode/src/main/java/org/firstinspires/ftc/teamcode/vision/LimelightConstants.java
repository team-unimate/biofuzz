package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LimelightConstants {
    public static String DEVICE_NAME = "limelight";

    public static int APRILTAG_PIPELINE_INDEX = 0;

    public static int POLL_RATE_HZ = 100;

    public static double CAM_FWD = 0;
    public static double CAM_LEFT = 0;
    public static double CAM_UP = 0;
    public static double CAM_PITCH_DEG = 0;
    public static double CAM_YAW_DEG = 0;

    public static double MAX_AGE_MS = 100;

    public static double STATE_MATCH_MAX_IN = 8;
    public static double STATE_TIE_MARGIN_IN = 2;
    public static double STATE_YAW_MARGIN_DEG = 30;
}
