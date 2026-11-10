package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;

/**
 * What the last AUTO left behind, for the TeleOp that follows it in the same match. Static
 * because the two OpModes share nothing else; it does not survive a Robot Controller restart.
 */
public final class AutoHandoff {
    // AUTO to TELEOP is an 8 s transition; anything much older belongs to another run.
    private static final long MAX_AGE_MS = 30_000;
    private static AutoHandoff last;
    private static long recordedMs;

    /** Field pose (frame documented in {@link Alliance}) where AUTO ended. */
    public final Pose pose;
    public final Alliance alliance;

    private AutoHandoff(Pose pose, Alliance alliance) {
        this.pose = pose;
        this.alliance = alliance;
    }

    /** Called when an AUTO ends. A null pose (no localizer data) records nothing. */
    public static synchronized void record(Pose pose, Alliance alliance) {
        last = pose == null ? null : new AutoHandoff(pose, alliance);
        recordedMs = System.currentTimeMillis();
    }

    /** Returns the last AUTO's result once, or null if there is none or it is stale. */
    public static synchronized AutoHandoff take() {
        AutoHandoff handoff = last;
        last = null;
        if (handoff == null || System.currentTimeMillis() - recordedMs > MAX_AGE_MS) {
            return null;
        }
        return handoff;
    }

    /** Robot heading at the end of AUTO relative to the direction the drivers face, radians. */
    public double headingFromDriverForward() {
        return pose.heading() - alliance.driverForwardHeading();
    }
}
