package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.jetbrains.annotations.NotNull;

import java.util.List;

/**
 * Reads Limelight targets; the OpMode chooses the pipeline and when to aim or shoot.
 *
 * <p>Shootability assumes a lowered CELL hides its AprilTags at shooting positions.
 * A visible tag therefore indicates a shootable CELL; no friendly tags means unknown.
 */
@Configurable
public final class Vision extends SubsystemBase {
    // Pipeline IDs must match the configuration at limelight.local:5801.
    public static int PIPELINE_APRILTAG = 0;
    public static int PIPELINE_POLLEN = 1;
    public static int PIPELINE_NECTAR_RED = 2;
    public static int PIPELINE_NECTAR_BLUE = 3;

    public static String LIMELIGHT_NAME = "limelight";

    // AprilTag area is a percentage of the image; aiming tolerance is in degrees.
    public static double APRILTAG_MIN_AREA = 0.1;
    public static double AIM_TOLERANCE_TX = 1.0;

    // Inclusive HIVE tag ID ranges for each alliance and CELL.
    public static int RED_A_MIN = 30;
    public static int RED_A_MAX = 33;
    public static int RED_B_MIN = 34;
    public static int RED_B_MAX = 37;
    public static int BLUE_A_MIN = 38;
    public static int BLUE_A_MAX = 41;
    public static int BLUE_B_MIN = 42;
    public static int BLUE_B_MAX = 45;

    // Color target area is a percentage of the image; offsets are in degrees.
    public static double ELEMENT_MIN_AREA = 0.05;
    public static double ELEMENT_MAX_OFFSET_DEG = 25.0;

    private final Limelight3A limelight;
    private LLResult result;
    private int activePipeline;

    public Vision(@NotNull final HardwareMap hMap) {
        limelight = hMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.start();
        activePipeline = PIPELINE_APRILTAG;
        limelight.pipelineSwitch(activePipeline);
    }

    @Override
    public void periodic() {
        result = limelight.getLatestResult();
    }

    public void setPipeline(int id) {
        if (id == activePipeline) {
            return;
        }
        activePipeline = id;
        limelight.pipelineSwitch(id);
    }

    public void aimMode() {
        setPipeline(PIPELINE_APRILTAG);
    }

    public void pollenMode() {
        setPipeline(PIPELINE_POLLEN);
    }

    public void nectarMode(boolean redTeam) {
        setPipeline(redTeam ? PIPELINE_NECTAR_RED : PIPELINE_NECTAR_BLUE);
    }

    public enum Cell {
        A,
        B
    }

    /**
     * True if the requested CELL is UP (its tags are visible).
     * False if it is DOWN or unknown.
     * Callers that need to distinguish DOWN from UNKNOWN can check
     * hasAnyHiveTag() separately.
     */
    public boolean isShootable(boolean redTeam, Cell cell) {
        return hasTag(minTagId(redTeam, cell), maxTagId(redTeam, cell));
    }

    /** True if any friendly HIVE tag is visible at all. */
    public boolean hasAnyHiveTag(boolean redTeam) {
        return hasTag(redTeam ? RED_A_MIN : BLUE_A_MIN, redTeam ? RED_B_MAX : BLUE_B_MAX);
    }

    /** Which CELL is shootable, or null if none/ambiguous. */
    public Cell shootableCell(boolean redTeam) {
        boolean cellAShootable = isShootable(redTeam, Cell.A);
        boolean cellBShootable = isShootable(redTeam, Cell.B);
        if (cellAShootable && !cellBShootable) {
            return Cell.A;
        }
        if (cellBShootable && !cellAShootable) {
            return Cell.B;
        }
        return null;
    }

    /** Horizontal offset in degrees to the largest qualifying CELL tag, or NaN if unavailable. */
    public double cellTx(boolean redTeam, Cell cell) {
        LLResultTypes.FiducialResult tag = largestTag(minTagId(redTeam, cell), maxTagId(redTeam, cell));
        return tag != null ? tag.getTargetXDegrees() : Double.NaN;
    }

    public boolean aimedAt(boolean redTeam, Cell cell) {
        double tx = cellTx(redTeam, cell);
        return !Double.isNaN(tx) && Math.abs(tx) <= AIM_TOLERANCE_TX;
    }

    /** Color target closest to the image centre horizontally within the area and offset limits. */
    public LLResultTypes.ColorResult bestElement() {
        if (!hasTarget()) {
            return null;
        }
        List<LLResultTypes.ColorResult> targets = result.getColorResults();
        if (targets == null) {
            return null;
        }
        LLResultTypes.ColorResult best = null;
        double bestOffset = Double.MAX_VALUE;
        for (LLResultTypes.ColorResult target : targets) {
            if (target.getTargetArea() < ELEMENT_MIN_AREA) {
                continue;
            }
            double offset = Math.abs(target.getTargetXDegrees());
            if (offset > ELEMENT_MAX_OFFSET_DEG
                    || Math.abs(target.getTargetYDegrees()) > ELEMENT_MAX_OFFSET_DEG) {
                continue;
            }
            if (offset < bestOffset) {
                bestOffset = offset;
                best = target;
            }
        }
        return best;
    }

    public boolean hasElement() {
        return bestElement() != null;
    }

    /** Horizontal offset in degrees to the best color target, or NaN if unavailable. */
    public double elementTx() {
        LLResultTypes.ColorResult element = bestElement();
        return element != null ? element.getTargetXDegrees() : Double.NaN;
    }

    /** True when the latest cached result is valid, regardless of target type. */
    public boolean hasTarget() {
        return result != null && result.isValid();
    }

    private int minTagId(boolean redTeam, Cell cell) {
        if (redTeam) {
            return cell == Cell.A ? RED_A_MIN : RED_B_MIN;
        }
        return cell == Cell.A ? BLUE_A_MIN : BLUE_B_MIN;
    }

    private int maxTagId(boolean redTeam, Cell cell) {
        if (redTeam) {
            return cell == Cell.A ? RED_A_MAX : RED_B_MAX;
        }
        return cell == Cell.A ? BLUE_A_MAX : BLUE_B_MAX;
    }

    private boolean hasTag(int minId, int maxId) {
        if (!hasTarget()) {
            return false;
        }
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null) {
            return false;
        }
        for (LLResultTypes.FiducialResult tag : tags) {
            int id = tag.getFiducialId();
            if (id >= minId && id <= maxId && tag.getTargetArea() >= APRILTAG_MIN_AREA) {
                return true;
            }
        }
        return false;
    }

    /** Largest qualifying tag in the inclusive ID range, or null if unavailable. */
    private LLResultTypes.FiducialResult largestTag(int minId, int maxId) {
        if (!hasTarget()) {
            return null;
        }
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null) {
            return null;
        }
        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult tag : tags) {
            int id = tag.getFiducialId();
            if (id < minId || id > maxId || tag.getTargetArea() < APRILTAG_MIN_AREA) {
                continue;
            }
            if (best == null || tag.getTargetArea() > best.getTargetArea()) {
                best = tag;
            }
        }
        return best;
    }
}
