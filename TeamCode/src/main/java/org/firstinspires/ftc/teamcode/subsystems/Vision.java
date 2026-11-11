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
 * BIOBUZZ Vision.
 *
 * Empirical rule: at shooting positions, a DOWN-tilted CELL has its
 * AprilTags occluded. So: target-side tag visible => shootable.
 * No tag but other side visible => DOWN. No friendly tags at all
 * => camera not on HIVE (UNKNOWN).
 */
@Configurable
public final class Vision extends SubsystemBase {

    // ---- Pipeline IDs (match limelight.local:5801) ----
    public static int PIPELINE_APRILTAG    = 0;
    public static int PIPELINE_POLLEN      = 1;
    public static int PIPELINE_NECTAR_RED  = 2;
    public static int PIPELINE_NECTAR_BLUE = 3;

    public static String LIMELIGHT_NAME = "limelight";

    // ---- Aiming ----
    public static double APRILTAG_MIN_AREA = 0.1;
    public static double AIM_TOLERANCE_TX  = 1.0;

    // ---- HIVE tag ID ranges, side A and B ----
    public static int RED_A_MIN   = 30, RED_A_MAX   = 33;
    public static int RED_B_MIN   = 34, RED_B_MAX   = 37;
    public static int BLUE_A_MIN  = 38, BLUE_A_MAX  = 41;
    public static int BLUE_B_MIN  = 42, BLUE_B_MAX  = 45;

    // ---- Element detection ----
    public static double ELEMENT_MIN_AREA       = 0.05;
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

    // ---- Pipeline ----
    public void setPipeline(int id) {
        if (id == activePipeline) return;
        activePipeline = id;
        limelight.pipelineSwitch(id);
    }

    public void aimMode()    { setPipeline(PIPELINE_APRILTAG); }
    public void pollenMode() { setPipeline(PIPELINE_POLLEN); }
    public void nectarMode(boolean redTeam) {
        setPipeline(redTeam ? PIPELINE_NECTAR_RED : PIPELINE_NECTAR_BLUE);
    }

    // ---- AprilTag helpers ----
    private boolean hasTag(int minId, int maxId) {
        if (result == null || !result.isValid()) return false;
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null) return false;
        for (LLResultTypes.FiducialResult t : tags) {
            int id = t.getFiducialId();
            if (id >= minId && id <= maxId && t.getTargetArea() >= APRILTAG_MIN_AREA)
                return true;
        }
        return false;
    }

    /** Largest visible tag in the given ID range, or null. */
    private LLResultTypes.FiducialResult largestTag(int minId, int maxId) {
        if (result == null || !result.isValid()) return null;
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null) return null;
        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult t : tags) {
            int id = t.getFiducialId();
            if (id < minId || id > maxId) continue;
            if (t.getTargetArea() < APRILTAG_MIN_AREA) continue;
            if (best == null || t.getTargetArea() > best.getTargetArea()) best = t;
        }
        return best;
    }

    // ---- Shootability (simplified to boolean + side enum) ----

    public enum Cell { A, B }

    /**
     * True if the requested CELL is UP (its tags are visible).
     * False if it is DOWN or unknown.
     * Callers that need to distinguish DOWN from UNKNOWN can check
     * hasAnyHiveTag() separately.
     */
    public boolean isShootable(boolean redTeam, Cell cell) {
        return hasTag(
                redTeam ? (cell == Cell.A ? RED_A_MIN  : RED_B_MIN) : (cell == Cell.A ? BLUE_A_MIN : BLUE_B_MIN),
                redTeam ? (cell == Cell.A ? RED_A_MAX  : RED_B_MAX) : (cell == Cell.A ? BLUE_A_MAX : BLUE_B_MAX)
        );
    }

    /** True if any friendly HIVE tag is visible at all. */
    public boolean hasAnyHiveTag(boolean redTeam) {
        return hasTag(redTeam ? RED_A_MIN : BLUE_A_MIN, redTeam ? RED_B_MAX : BLUE_B_MAX);
    }

    /** Which CELL is shootable, or null if none/ambiguous. */
    public Cell shootableCell(boolean redTeam) {
        boolean a = isShootable(redTeam, Cell.A);
        boolean b = isShootable(redTeam, Cell.B);
        if (a && !b) return Cell.A;
        if (b && !a) return Cell.B;
        return null;
    }

    // ---- Aiming ----

    public double cellTx(boolean redTeam, Cell cell) {
        LLResultTypes.FiducialResult t = largestTag(
                redTeam ? (cell == Cell.A ? RED_A_MIN  : RED_B_MIN)  : (cell == Cell.A ? BLUE_A_MIN : BLUE_B_MIN),
                redTeam ? (cell == Cell.A ? RED_A_MAX  : RED_B_MAX)  : (cell == Cell.A ? BLUE_A_MAX : BLUE_B_MAX)
        );
        return t != null ? t.getTargetXDegrees() : Double.NaN;
    }

    public boolean aimedAt(boolean redTeam, Cell cell) {
        double tx = cellTx(redTeam, cell);
        return !Double.isNaN(tx) && Math.abs(tx) <= AIM_TOLERANCE_TX;
    }

    // ---- Elements (color pipelines) ----

    public LLResultTypes.ColorResult bestElement() {
        if (result == null || !result.isValid()) return null;
        List<LLResultTypes.ColorResult> blobs = result.getColorResults();
        if (blobs == null) return null;
        LLResultTypes.ColorResult best = null;
        double bestOff = Double.MAX_VALUE;
        for (LLResultTypes.ColorResult b : blobs) {
            if (b.getTargetArea() < ELEMENT_MIN_AREA) continue;
            double tx = b.getTargetXDegrees();
            if (Math.abs(tx) > ELEMENT_MAX_OFFSET_DEG) continue;
            if (Math.abs(b.getTargetYDegrees()) > ELEMENT_MAX_OFFSET_DEG) continue;
            if (Math.abs(tx) < bestOff) { bestOff = Math.abs(tx); best = b; }
        }
        return best;
    }

    public boolean hasElement() {
        return bestElement() != null;
    }
    public double elementTx() {
        LLResultTypes.ColorResult e = bestElement();
        return e != null ? e.getTargetXDegrees() : Double.NaN;
    }

    public boolean hasTarget() {
        return result != null && result.isValid();
    }
}