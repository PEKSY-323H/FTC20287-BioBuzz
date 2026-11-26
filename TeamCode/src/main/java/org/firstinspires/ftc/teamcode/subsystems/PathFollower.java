package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.OctoQuadImuBackupLocalizer;
import org.jetbrains.annotations.NotNull;

/**
 * Owns the Pedro follower (drive motors + OctoQuad with IMU heading backup). Do not construct
 * {@link Drive} or {@link Localizer} in the same OpMode: they claim the same hardware.
 */
public final class PathFollower extends SubsystemBase {
    private final Follower follower;
    private final OctoQuadImuBackupLocalizer localization;

    public PathFollower(@NotNull final HardwareMap hMap, @NotNull final Pose startPose) {
        localization = Constants.createBackupLocalizer(hMap);
        follower = Constants.create(hMap, localization);
        follower.setPose(startPose);
    }

    /** Which sensor the pose comes from, for telemetry and safety checks. */
    public OctoQuadImuBackupLocalizer getLocalization() {
        return localization;
    }

    /** Raw follower for SolversLib's pedroCommand classes (FollowPathCommand, TurnToCommand...). */
    public Follower getFollower() {
        return follower;
    }

    /** The scheduler calls this once per loop, before any command's execute(). */
    @Override
    public void periodic() {
        follower.update();
    }

    /** Actively holds a pose (e.g. while shooting) until the next follow() or stop(). */
    public void hold(@NotNull final Pose pose) {
        follower.hold(pose);
    }

    /** Drops any path/hold and zeroes the drive motors immediately. */
    public void stop() {
        follower.stop();
        follower.drivetrain.stop();
    }

    public Pose getPose() {
        return follower.pose();
    }

    public void setPose(@NotNull final Pose pose) {
        follower.setPose(pose);
    }

    public Follower.Mode getMode() {
        return follower.mode();
    }

    /** Fraction of the current path completed, 0..1. */
    public double getCompletion() {
        return follower.completion();
    }
}
