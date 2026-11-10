package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.jetbrains.annotations.NotNull;

/**
 * Reads the OctoQuad MK2 heading (radians, CCW-positive) and reports it relative to a
 * driver-forward reference: the facing captured at START, a re-zero, or an AUTO handoff.
 */
public final class Localizer extends SubsystemBase {
    // One bad I2C packet must not brake the drivetrain: reuse the last good heading for
    // this long, then report the heading as unavailable.
    public static final double HEADING_HOLD_SECONDS = 0.1;

    private final OctoQuad octoQuad;
    private final OctoQuad.LocalizerDataBlock data = new OctoQuad.LocalizerDataBlock();
    private final ElapsedTime sinceValid = new ElapsedTime();
    private boolean valid;
    // Last good device heading, or NaN once it is older than HEADING_HOLD_SECONDS.
    private double rawHeadingRadians = Double.NaN;
    // Device heading that counts as driver-forward; NaN until a reference is set.
    private double startHeadingRadians = Double.NaN;
    private int badCrcPackets;

    public Localizer(@NotNull final HardwareMap hMap) {
        this.octoQuad = hMap.get(OctoQuad.class, "octoquad");
        // Recalibrate local IMU state without changing saved pod geometry or heading scale.
        this.octoQuad.resetLocalizerAndCalibrateIMU();
    }

    @Override
    public void periodic() {
        octoQuad.readLocalizerData(data);
        if (!data.crcOk) {
            badCrcPackets++;
        }
        valid = data.isDataValid() && Float.isFinite(data.heading_rad);
        if (valid) {
            rawHeadingRadians = data.heading_rad;
            sinceValid.reset();
        } else if (sinceValid.seconds() > HEADING_HOLD_SECONDS) {
            rawHeadingRadians = Double.NaN;
        }
    }

    /** Heading relative to the driver-forward reference in (-pi, pi], or NaN if unavailable. */
    public double getHeadingRadians() {
        if (!Double.isFinite(rawHeadingRadians) || !Double.isFinite(startHeadingRadians)) {
            return Double.NaN;
        }
        double relativeHeading = rawHeadingRadians - startHeadingRadians;
        return Math.atan2(Math.sin(relativeHeading), Math.cos(relativeHeading));
    }

    /** Device heading, unaffected by reference changes; NaN if unavailable. Not wrapped. */
    public double getRawHeadingRadians() {
        return rawHeadingRadians;
    }

    /** Uses the latest sample as driver-forward, unless a reference already exists. */
    public void captureStartHeading() {
        if (valid && !hasReference()) {
            startHeadingRadians = rawHeadingRadians;
        }
    }

    /**
     * Moves the reference so the robot's current facing reads {@code headingRadians}
     * (0 = facing driver-forward). Returns false if no heading is available yet.
     */
    public boolean resetHeading(double headingRadians) {
        if (!Double.isFinite(rawHeadingRadians)) {
            return false;
        }
        startHeadingRadians = rawHeadingRadians - headingRadians;
        return true;
    }

    public boolean hasReference() {
        return Double.isFinite(startHeadingRadians);
    }

    /** True when the latest packet itself was good (no held value). */
    public boolean isValid() {
        return valid;
    }

    public String getStatus() {
        if (!data.crcOk) {
            return "INVALID CRC";
        }
        if (!Float.isFinite(data.heading_rad)) {
            return "INVALID HEADING";
        }
        return String.valueOf(data.localizerStatus);
    }

    public int getBadCrcPackets() {
        return badCrcPackets;
    }
}
