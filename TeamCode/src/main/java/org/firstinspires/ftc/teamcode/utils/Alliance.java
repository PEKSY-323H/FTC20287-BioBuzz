package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;

/**
 * Alliance-dependent BIOBUZZ (2026-2027) field facts, from Competition Manual TU08.
 *
 * <p>Team field frame used for every Pedro pose (inches, radians, heading CCW-positive):
 * origin at the audience-side corner next to the RED alliance area, +x toward the BLUE
 * alliance wall (tile columns A to F), +y away from the audience (tile rows 1 to 6),
 * heading 0 faces +x. RED owns x &lt; 72 (columns A-C), BLUE owns x &gt; 72 (columns D-F).
 * This frame is a team convention, not an FTC or Limelight field frame: convert explicitly
 * before exchanging poses with either.
 */
public enum Alliance {
    // Section 9.9: tags 30-37 are under the two red CELLS, 38-45 under the two blue CELLS.
    RED(30, 37, 0.0),
    BLUE(38, 45, Math.PI);

    /** Field edge length in inches (section 9.2). */
    public static final double FIELD_SIZE = 144.0;
    /** Each CELL carries one cluster of this many AprilTags with consecutive IDs. */
    public static final int TAGS_PER_CELL = 4;
    /** The HIVE structure holds four CELLS: two red, two blue. */
    public static final int CELL_COUNT = 4;
    private static final int FIRST_CELL_TAG_ID = 30;

    private final int firstTagId;
    private final int lastTagId;
    private final double driverForwardHeading;

    Alliance(int firstTagId, int lastTagId, double driverForwardHeading) {
        this.firstTagId = firstTagId;
        this.lastTagId = lastTagId;
        this.driverForwardHeading = driverForwardHeading;
    }

    /**
     * Converts a pose authored for BLUE to this alliance. The BIOBUZZ field is 180-degree
     * rotationally symmetric (LOADING ZONES on tiles A5 and F2, GARDENS in opposite
     * corners), so RED is BLUE rotated about the field centre, not mirrored.
     */
    public Pose fromBlue(Pose bluePose) {
        if (this == BLUE) {
            return bluePose;
        }
        return new Pose(FIELD_SIZE - bluePose.x(), FIELD_SIZE - bluePose.y(),
                headingFromBlue(bluePose.heading()));
    }

    /** Converts a BLUE field heading in radians to this alliance, wrapped to (-pi, pi]. */
    public double headingFromBlue(double blueHeading) {
        double heading = this == BLUE ? blueHeading : blueHeading + Math.PI;
        return Math.atan2(Math.sin(heading), Math.cos(heading));
    }

    /** Field heading the drive team faces from its ALLIANCE AREA (RED is on the audience's left). */
    public double driverForwardHeading() {
        return driverForwardHeading;
    }

    /** True for the AprilTags under this alliance's own CELLS. */
    public boolean ownsTag(int tagId) {
        return tagId >= firstTagId && tagId <= lastTagId;
    }

    /** Index 0..3 of the CELL a tag belongs to, or -1 if it is not a CELL tag. */
    public static int cellOfTag(int tagId) {
        int cell = Math.floorDiv(tagId - FIRST_CELL_TAG_ID, TAGS_PER_CELL);
        return cell >= 0 && cell < CELL_COUNT ? cell : -1;
    }
}
