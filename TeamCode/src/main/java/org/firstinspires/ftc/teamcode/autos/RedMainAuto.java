package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.utils.Alliance;

/**
 * RED main autonomous: drives the ten paths of {@code Auto routines/Auto red v1.pp.json}
 * in order. Every coordinate below is copied from that file (RED side, team field frame
 * documented in {@link Alliance}), so the numbers can be compared with the visualizer.
 *
 * <p>The path file has no waits or event markers, so this routine only drives: the
 * shooter, intake and gate are never commanded. Add mechanism commands between the
 * {@code follow(...)} steps in {@link #buildRoutine()}.
 *
 * <p>Not run on the robot yet, and {@code pedro/Constants} still holds placeholder
 * localizer and Foresight values; tune those before driving this.
 */
@Configurable
@Autonomous(name = "Red Main Auto", group = "Red", preselectTeleOp = "Field-Centric")
public final class RedMainAuto extends BaseAuto {
    // Paths 1-9 are cut off once this much of AUTO is left, so the last path (into the
    // LOADING ZONE, PARK) always gets its time. Estimated from its 53 in length, not measured.
    public static double PARK_RESERVE_SECONDS = 4.0;

    public RedMainAuto() {
        super(Alliance.RED);
    }

    /**
     * The file's start point says heading 90, but Path 1 starts at 270 and Path 3 returns
     * to this spot at 270; the paths win. Place the robot facing the audience (-y).
     */
    @Override
    protected Pose startPose() {
        return red(54.977, 16.990, 270);
    }

    @Override
    protected Command buildRoutine() {
        Pose start = startPose();
        Pose lowApproach = red(18.692, 6.459, 180);
        Pose lowWall = red(5.549, 6.465, 180);
        Pose backAtStart = red(55.068, 17.188, 270);
        Pose flowerApproach = red(14.038, 46.315, 180);
        // Red Alliance FLOWER is at x 0-3.8, y 43.65-50.35.
        Pose redFlower = red(5.860, 46.808, 180);
        Pose flowerBackOff = red(19.842, 46.533, 180);
        Pose farStaging = red(58.621, 124.333, 90);
        // Scoring-Side FLOWER is at x 43.4-50.1, y 137.7-141.5.
        Pose scoringFlower = red(47.124, 134.089, 90);
        Pose farStagingReturn = red(58.653, 124.364, 90);
        // Tile A5, the RED LOADING ZONE.
        Pose park = red(8.175, 108.983, 0);

        Command paths1to9 = new SequentialCommandGroup(
                follow(curve(start, red(38.892, 31.829, 0), lowApproach)),
                follow(line(lowApproach, lowWall)),
                follow(curve(lowWall, red(38.562, 32.095, 0), backAtStart)),
                follow(curve(backAtStart, red(38.977, 40.912, 0), flowerApproach)),
                follow(line(flowerApproach, redFlower)),
                follow(line(redFlower, flowerBackOff)),
                follow(curve(flowerBackOff, red(20.212, 114.052, 0), farStaging)),
                follow(curve(farStaging, red(45.537, 117.725, 0), scoringFlower)),
                follow(curve(scoringFlower, red(45.299, 117.510, 0), farStagingReturn)));

        return new SequentialCommandGroup(
                untilSecondsLeft(paths1to9, PARK_RESERVE_SECONDS),
                // Starts from wherever the robot was cut off; Pedro steers onto the line.
                follow(line(farStagingReturn, park)));
    }

    /**
     * A pose as drawn in the path file (RED side). {@link #pose} expects the BLUE
     * equivalent, which is the same pose rotated 180 degrees about the field centre.
     */
    private Pose red(double x, double y, double headingDegrees) {
        return pose(Alliance.FIELD_SIZE - x, Alliance.FIELD_SIZE - y, headingDegrees + 180);
    }
}
