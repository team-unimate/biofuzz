package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;


public final class Clock {
    private static final ElapsedTime timer = new ElapsedTime();

    public static double seconds() {
        return timer.seconds();
    }

    private Clock() {
    }
}
