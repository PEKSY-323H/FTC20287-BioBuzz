package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.jetbrains.annotations.NotNull;

/**
 * Standalone Pedro OctoQuad localization in the same inches/radians field frame as AUTO.
 * Do not construct alongside a PathFollower or Follower: they own their own localizer.
 */
public final class Localizer extends SubsystemBase {
    private final OctoQuadLocalizer localizer;

    public Localizer(@NotNull final HardwareMap hardwareMap) {
        localizer = Constants.createLocalizer(hardwareMap);
    }

    @Override
    public void periodic() {
        localizer.update();
    }

    /** Current field heading, CCW-positive in radians. */
    public double getHeadingRadians() {
        return localizer.pose().heading();
    }

    public void setFieldPose(@NotNull Pose pose) {
        localizer.setPose(pose);
    }

    @NotNull
    public Pose getFieldPose() {
        return localizer.pose();
    }
}
