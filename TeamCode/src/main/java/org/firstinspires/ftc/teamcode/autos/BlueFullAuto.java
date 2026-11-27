package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autos.Points.Blue;
import org.firstinspires.ftc.teamcode.utils.Alliance;

/**
 * Blue max routine: the Red Full Auto turned 180 degrees about the Pedro field centre
 * (70.75, 70.75), so every BLUE point is (141.5 - x, 141.5 - y, heading + 180) of the RED one.
 * Field points live in {@link Points.Blue} (field frame, BLUE side):
 * <ol>
 * <li>START → SHOOT_1, wait until the robot is on the point, shoot the preload.</li>
 * <li>SHOOT_1 → FLOWER_PREP_1 lowering the splitter on the way; FLOWER_PREP_1 → FLOWER_1,
 *     collect.</li>
 * <li>FLOWER_1 → SHOOT_2; wait until on the point, wait WAIT_BEFORE_SHOOT_2_SECONDS, shoot.</li>
 * <li>SHOOT_2 → FLOWER_PREP_2 → FLOWER_2, collect.</li>
 * <li>FLOWER_2 → SHOOT_2; wait until on the point, wait WAIT_BEFORE_SHOOT_2_SECONDS, shoot.</li>
 * <li>SHOOT_2 → STOP_1 (PARK).</li>
 * </ol>
 * The splitter stays down through collection and retracts when the final park path starts.
 * The intake runs during collection and shooting, and slowly while carrying pollen. Everything before the park drive is cut off
 * PARK_RESERVE_SECONDS before the end so PARK always gets its time.
 */
@Configurable
@Autonomous(name = "Blue Full Auto", group = "Blue", preselectTeleOp = "Field-Centric")
public final class BlueFullAuto extends BaseAuto {
    public static double SHOOT_SECONDS = 2;
    // Time stopped at each FLOWER with the intake on (the splitter is already down by then).
    public static double FLOWER_SECONDS = 2.0;
    // Wait at shooting position 2, once the robot is on the point, before each shot there.
    public static double WAIT_BEFORE_SHOOT_2_SECONDS = 1.0;
    public static long LONG_PATH_TIMEOUT_MS = 8000;
    // Whatever is still running is cut off this long before the end of AUTO, so the drive to
    // PARK always gets its time.
    public static double PARK_RESERVE_SECONDS = 4.0;

    // Like the TeleOp's splitterDeployed: whether the last splitter move started was down.
    private boolean splitterDown;

    public BlueFullAuto() {
        super(Alliance.BLUE);
    }

    @Override
    protected Pose startPose() {
        return bluePose(Blue.START);
    }

    @Override
    protected Command buildRoutine() {
        splitterDown = false;
        Pose start1 = startPose();
        Pose shoot1 = bluePose(Blue.SHOOT_1);
        Pose flowerPrep1 = bluePose(Blue.FLOWER_PREP_1);
        Pose flower1 = bluePose(Blue.FLOWER_1);
        Pose shoot2 = bluePose(Blue.SHOOT_2);
        Pose flowerPrep2 = bluePose(Blue.FLOWER_PREP_2);
        Pose flower2 = bluePose(Blue.FLOWER_2);
        Pose park = bluePose(Blue.STOP_1);

        Path start1ToShoot1 = line(start1, shoot1);
        Path shoot1ToFlowerPrep1 = curve(shoot1, bluePose(Blue.SHOOT_1_TO_FLOWER_PREP_1_CTRL), flowerPrep1,
                Interpolator.linear(shoot1.heading(), flowerPrep1.heading()));
        Path flowerPrep1ToFlower1 = line(flowerPrep1, flower1,
                Interpolator.constant(flower1.heading()));
        Path flower1ToShoot2 = Paths.curve(flower1,
                bluePose(Blue.FLOWER_1_TO_SHOOT_2_CTRL_A),
                bluePose(Blue.FLOWER_1_TO_SHOOT_2_CTRL_B), shoot2)
                .heading(Interpolator.piecewise()
                        .until(0.3,
                                Interpolator.linear(flower1.heading(), shoot2.heading()))
                        .until(1.0, Interpolator.constant(shoot2.heading())));
        Path shoot2ToFlowerPrep2 = line(shoot2, flowerPrep2);
        Path flowerPrep2ToFlower2 = line(flowerPrep2, flower2,
                Interpolator.constant(flower2.heading()));
        // Headings go from each point's heading to the next one's.
        Path flower2ToShoot2 = line(flower2, shoot2);
        Path shoot2ToPark = line(shoot2, park);

        return new SequentialCommandGroup(
                untilSecondsLeft(new SequentialCommandGroup(
                        intakeSlow(),
                        // 1. Preload at shooting position 1.
                        spinUp(),
                        aimWhileShooting(shoot1,
                                follow(start1ToShoot1),
                                settleAt(shoot1),
                                shoot(SHOOT_SECONDS)),
                        // 2. FLOWER 1: splitter down on the way to the prep point.
                        lowerSplitterWhileFollowing(shoot1ToFlowerPrep1, DEFAULT_PATH_TIMEOUT_MS),
                        collectAtFlower(flowerPrep1ToFlower1),
                        // 3. Shooting position 2 (the splitter stays down).
                        spinUp(),
                        aimWhileShooting(shoot2,
                                follow(flower1ToShoot2, PATH_SPEED_FRACTION, LONG_PATH_TIMEOUT_MS),
                                settleAt(shoot2),
                                new WaitCommand((long) (WAIT_BEFORE_SHOOT_2_SECONDS * 1000)),
                                shoot(SHOOT_SECONDS)),
                        // 4. FLOWER 2 (the splitter is still down).
                        follow(shoot2ToFlowerPrep2),
                        collectAtFlower(flowerPrep2ToFlower2),
                        // 5. Back to shooting position 2.
                        spinUp(),
                        aimWhileShooting(shoot2,
                                follow(flower2ToShoot2),
                                settleAt(shoot2),
                                new WaitCommand((long) (WAIT_BEFORE_SHOOT_2_SECONDS * 1000)),
                                shoot(SHOOT_SECONDS))), PARK_RESERVE_SECONDS),
                // If the cutoff hit before a shoot() finished, the flywheel is still spinning.
                new InstantCommand(shooter::stop, shooter),
                intakeOff(),
                // 6. Retract while driving to PARK.
                retractSplitter(),
                follow(shoot2ToPark, PATH_SPEED_FRACTION, LONG_PATH_TIMEOUT_MS)) {
            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                // Stop the intake even if the routine was interrupted during collection.
                shooter.stop();
                intake.stop();
            }
        };
    }

    /**
     * Waits at the prep point until the splitter is fully down, then drives the last stretch to
     * a FLOWER and stays FLOWER_SECONDS with collection power, then slows for loaded transport.
     *
     * <p>There is no timeout on the splitter wait: if the splitter never lowers (for example
     * Intake.SPLITTER_DEPLOY_SECONDS is 0), the robot stays at the prep point until the
     * PARK_RESERVE_SECONDS cutoff and then parks, rather than driving into the FLOWER.
     */
    private Command collectAtFlower(Path path) {
        return new SequentialCommandGroup(
                // Never start the FLOWER approach before the splitter has finished going down.
                new WaitUntilCommand(() -> splitterDown && !intake.isSplitterMoving()),
                intakeOn(),
                follow(path),
                new WaitCommand((long) (FLOWER_SECONDS * 1000)),
                intakeSlow());
    }

    /** Starts lowering the splitter, then follows the path while it moves on its timer. */
    private Command lowerSplitterWhileFollowing(Path path, long timeoutMs) {
        return new SequentialCommandGroup(
                lowerSplitter(),
                follow(path, PATH_SPEED_FRACTION, timeoutMs));
    }

    /**
     * Lowers the splitter the same way the TeleOp's A press does: waits for any running splitter
     * move to finish (where the TeleOp would ignore the press), then lowers it only if it is not
     * already down; it only counts as down once the move started.
     */
    private Command lowerSplitter() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> !intake.isSplitterMoving()),
                new InstantCommand(() -> {
                    if (!splitterDown && intake.deploySplitter()) {
                        splitterDown = true;
                    }
                }, intake));
    }
}
