package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.utils.Alliance;

/**
 * RED main autonomous: drives the eleven paths of
 * {@code Auto routines/Auto red v1。1.pp.json} in order (an opening line off the wall,
 * then the ten called Path 1-10 here). Every coordinate below is copied from that file (RED side, team field frame
 * documented in {@link Alliance}), so the numbers can be compared with the visualizer.
 *
 * <p>Mechanism plan (the path file itself has no waits or markers):
 * <ol>
 * <li>Opening line from the wall to the shooting position, shoot the preload, Path 1.</li>
 * <li>Path 2 with the intake running.</li>
 * <li>Path 3, shoot, Path 4.</li>
 * <li>Path 5 with the intake running, then collect at the Red Alliance FLOWER.</li>
 * <li>Paths 6 and 7, shoot.</li>
 * <li>Path 8 with the intake running, then collect at the Scoring-Side FLOWER.</li>
 * <li>Path 9, shoot, Path 10 into the LOADING ZONE.</li>
 * </ol>
 *
 * <p>Not run on the robot yet. At the default path speed the plan needs more than 30 s
 * (16 s of feeding and 6 s at the FLOWERS before any driving), so the end of it is cut
 * off by PARK_RESERVE_SECONDS. {@code pedro/Constants} also still holds placeholder
 * localizer and Foresight values; tune those before driving this.
 */
@Configurable
@Autonomous(name = "Red Main Auto", group = "Red", preselectTeleOp = "Field-Centric")
public final class RedMainAuto extends BaseAuto {
    // Paths 1-9 and their shots are cut off once this much of AUTO is left, so the last
    // path (into the LOADING ZONE, PARK) always gets its time. Estimated from its 53 in
    // length, not measured.
    public static double PARK_RESERVE_SECONDS = 4.0;
    // Time the gate stays open and the intake feeds for each shot, after the flywheel is at
    // speed. The driver's estimate, not measured.
    public static double SHOOT_SECONDS = 4.0;
    // Time spent stopped at a FLOWER with the intake and the FLOWER servo running.
    public static double FLOWER_SECONDS = 3.0;

    public RedMainAuto() {
        super(Alliance.RED);
    }

    /**
     * Against the audience-side wall, at the position the drive team read off the path
     * visualizer (x 53.3, y 5.3, heading 270); the path file's own start point is
     * (58.855, 5.479). Place the robot facing the audience (-y).
     */
    @Override
    protected Pose startPose() {
        return red(53.3, 5.3, 270);
    }

    @Override
    protected Command buildRoutine() {
        Pose start = startPose();
        Pose shootingPosition = red(58.696, 17.430, 270);
        Pose lowApproach = red(18.692, 6.459, 180);
        Pose lowWall = red(5.549, 6.465, 180);
        Pose shootingReturn = red(58.277, 17.107, 270);
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

        // spinUp() before the path that leads to a shot brings the flywheel to speed on the
        // way, so the shot does not spend its time waiting for it.
        Command scoring = new SequentialCommandGroup(
                spinUp(),
                follow(line(start, shootingPosition)),
                shoot(SHOOT_SECONDS),
                follow(curve(shootingPosition, red(38.892, 31.829, 0), lowApproach)),
                intakeAlong(line(lowApproach, lowWall)),
                spinUp(),
                follow(curve(lowWall, red(38.562, 32.095, 0), shootingReturn)),
                shoot(SHOOT_SECONDS),
                follow(curve(shootingReturn, red(38.977, 40.912, 0), flowerApproach)),
                collectAtFlower(line(flowerApproach, redFlower)),
                follow(line(redFlower, flowerBackOff)),
                spinUp(),
                follow(curve(flowerBackOff, red(20.212, 114.052, 0), farStaging)),
                shoot(SHOOT_SECONDS),
                collectAtFlower(curve(farStaging, red(45.537, 117.725, 0), scoringFlower)),
                spinUp(),
                follow(curve(scoringFlower, red(45.299, 117.510, 0), farStagingReturn)),
                shoot(SHOOT_SECONDS)) {
            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                // A cut-off between spinUp() and its shot must not leave the flywheel on.
                shooter.stop();
            }
        };

        return new SequentialCommandGroup(
                untilSecondsLeft(scoring, PARK_RESERVE_SECONDS),
                // Starts from wherever the robot was cut off; Pedro steers onto the line.
                follow(line(farStagingReturn, park)));
    }

    /** Follows a path with the intake running; the intake stops even when interrupted. */
    private Command intakeAlong(Path path) {
        return new SequentialCommandGroup(intakeOn(), follow(path)) {
            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                intake.stop();
            }
        };
    }

    /**
     * Follows a path to a FLOWER with the intake running, then stays there for
     * FLOWER_SECONDS with the FLOWER servo started. Intake and servo stop even when
     * interrupted.
     */
    private Command collectAtFlower(Path path) {
        return new SequentialCommandGroup(
                intakeOn(),
                follow(path),
                new InstantCommand(this::startFlowerServo),
                new WaitCommand((long) (FLOWER_SECONDS * 1000))) {
            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                intake.stop();
                stopFlowerServo();
            }
        };
    }

    // TODO: the FLOWER intake servo has no subsystem yet; these two do nothing.
    private void startFlowerServo() {
    }

    private void stopFlowerServo() {
    }

    /**
     * A pose as drawn in the path file (RED side). {@link #pose} expects the BLUE
     * equivalent, which is the same pose rotated 180 degrees about the field centre.
     */
    private Pose red(double x, double y, double headingDegrees) {
        return pose(Alliance.FIELD_SIZE - x, Alliance.FIELD_SIZE - y, headingDegrees + 180);
    }
}
