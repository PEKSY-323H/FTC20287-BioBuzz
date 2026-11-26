package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.GamepadConfig;

/**
 * Field-centric manual drive: with AUTO handoff, stick-forward moves away from the alliance's
 * drivers (+x RED, -x BLUE). Without handoff, heading zero is forward for either alliance.
 * Pose, B/X paths, AUTO handoff and the Y reset come from {@link PedroDriveTeleOp}.
 */
@TeleOp(name = "Field-Centric")
public final class FieldCentricDrive extends PedroDriveTeleOp {
    private boolean robotCentric;
    private boolean driveModeWasHeld;

    @Override
    protected boolean updateDriveMode(GamepadConfig controls) {
        boolean held = controls.isDriveModeToggleHeld();
        boolean changed = held && !driveModeWasHeld;
        driveModeWasHeld = held;
        if (changed) {
            robotCentric = !robotCentric;
            controls.rumble(200);
        }
        return changed;
    }

    @Override
    protected void driveManually(double forward, double lateral, double turn) {
        if (robotCentric) {
            follower.manual(forward, lateral, turn);
        } else {
            follower.manual(ManualDrive.fieldCentric(forward, lateral, turn,
                    headingFromDriverForward()));
        }
    }

    @Override
    protected String stickFrameHint() {
        return (robotCentric ? "ROBOT: forward follows the robot."
                : isUsingAutoHandoff()
                        ? "FIELD: forward away from drivers (+x RED, -x BLUE)."
                        : "FIELD: standalone, heading zero is forward (+x).")
                + " D-pad right: toggle drive frame and cancel the current path.";
    }
}
