package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.GamepadConfig;
import org.firstinspires.ftc.teamcode.subsystems.Localizer;
import org.firstinspires.ftc.teamcode.utils.AutoHandoff;

/**
 * Field-centric (headless) mecanum drive.
 *
 * <p>Forward reference: straight after an AUTO it is taken from the AUTO's final pose and
 * alliance, so stick-forward is away from the drivers; otherwise it is the robot's facing
 * at START. BACK re-zeroes it to the current facing at any time. If the OctoQuad heading
 * drops out the sticks become robot-centric until it returns.
 */
@TeleOp(name = "Field-Centric")
public final class FieldCentricDrive extends BaseDriveTeleOp {
    private Localizer localizer;
    private GamepadConfig controls;
    // Robot heading relative to driver-forward at INIT, from the last AUTO; NaN if none.
    private double handoffHeadingRadians;
    private boolean robotCentricFallback;

    @Override
    protected DriveCommand createDriveCommand(Drive drive, GamepadConfig controls) {
        this.controls = controls;
        localizer = new Localizer(hardwareMap);
        robotCentricFallback = false;
        handoffHeadingRadians = takeAutoHandoff();
        return new DriveCommand(
                drive,
                controls::getForward,
                controls::getStrafe,
                controls::getTurn,
                localizer,
                GamepadConfig.DRIVE_DEADBAND,
                GamepadConfig.DRIVE_LIMIT,
                GamepadConfig.TURN_LIMIT);
    }

    /**
     * Consumes the last AUTO's result. The OctoQuad heading was just zeroed with the robot
     * still standing where AUTO left it, so that pose gives its heading relative to
     * driver-forward. Returns NaN when no AUTO ran just before this TeleOp.
     */
    private double takeAutoHandoff() {
        AutoHandoff handoff = AutoHandoff.take();
        return handoff == null ? Double.NaN : handoff.headingFromDriverForward();
    }

    /** Sets the forward reference as soon as a heading sample exists; then does nothing. */
    private void setForwardReference(boolean started) {
        if (localizer.hasReference()) {
            return;
        }
        if (Double.isFinite(handoffHeadingRadians)) {
            localizer.resetHeading(handoffHeadingRadians);
        } else if (started) {
            localizer.captureStartHeading();
        }
    }

    @Override
    protected void updateDriveInitialization() {
        localizer.periodic();
        setForwardReference(false);

        telemetry.addData("OctoQuad", localizer.getStatus());
        telemetry.addLine("Keep still; wait for RUNNING before START.");
        if (Double.isFinite(handoffHeadingRadians)) {
            telemetry.addLine("Forward = away from the drivers (from AUTO). BACK re-zeroes.");
        } else {
            telemetry.addLine("START locks this facing as forward. BACK re-zeroes.");
        }
    }

    @Override
    protected void beforeDriveRun() {
        if (!localizer.hasReference()) {
            // Needs a fresh sample before the scheduler's first run (preRun() is not used here).
            localizer.periodic();
            setForwardReference(true);
        }
        if (controls.isResetHeadingHeld()) {
            localizer.resetHeading(0.0);
        }
    }

    @Override
    protected void addDriveTelemetry() {
        boolean fallback = !Double.isFinite(localizer.getHeadingRadians());
        if (fallback && !robotCentricFallback) {
            // The driver is not looking at the Driver Station when the mode changes.
            controls.rumble(600);
        }
        robotCentricFallback = fallback;

        if (fallback) {
            telemetry.addData("Drive", "ROBOT-CENTRIC: heading unavailable");
        } else {
            telemetry.addData("Drive", "Field reference");
        }
        telemetry.addData("OctoQuad", localizer.getStatus());
        telemetry.addData("Heading from forward (deg)", Math.toDegrees(localizer.getHeadingRadians()));
        telemetry.addData("Bad CRC packets", localizer.getBadCrcPackets());
        // Position counts from this OpMode's INIT, not from the field origin.
        telemetry.addData("OctoQuad packet valid", localizer.isValid());
        telemetry.addData("OctoQuad x (in)", localizer.getXInches());
        telemetry.addData("OctoQuad y (in)", localizer.getYInches());
        telemetry.addData("OctoQuad heading (deg)", Math.toDegrees(localizer.getRawHeadingRadians()));
        telemetry.addData("OctoQuad x velocity (in/s)", localizer.getXVelocityInches());
        telemetry.addData("OctoQuad y velocity (in/s)", localizer.getYVelocityInches());
        telemetry.addData("OctoQuad rotation (deg/s)",
                Math.toDegrees(localizer.getHeadingVelocityRadians()));
    }
}
