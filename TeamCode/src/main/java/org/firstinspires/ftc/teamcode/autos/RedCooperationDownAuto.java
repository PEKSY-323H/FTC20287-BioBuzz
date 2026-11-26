package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autos.Points.Red;
import org.firstinspires.ftc.teamcode.utils.Alliance;

/**
 * RedCooperationDown.pp.json route using the shared, tuned {@link Points.Red} waypoints.
 * Shoots the preload, collects the GARDEN with the splitter retracted, returns to SHOOT_1
 * to wait for the partner and shoot, then collects FLOWER 1, returns to shoot, and parks
 * at STOP_1. The splitter deploys on the FLOWER approach and retracts as parking starts.
 * The intake runs during collection and shooting, and slowly while carrying pollen. Collection, waits and shooting are
 * interrupted with PARK_RESERVE_SECONDS left so retraction and the final path can start.
 *
 * <p>The route editor has slightly different coordinates for repeated SHOOT_1 endpoints;
 * all visits use the shared SHOOT_1 instead. Its start heading is 90, but its first path
 * holds -90; the shared START heading of 270 matches that first path.
 */
@Configurable
@Autonomous(name = "Red Cooperation Down Auto", group = "Red", preselectTeleOp = "Field-Centric")
public final class RedCooperationDownAuto extends BaseAuto {
    public static double SHOOT_SECONDS = 2.0;
    public static double FLOWER_SECONDS = 2.0;
    public static double GARDEN_SECONDS = 2.0;
    // Set in Panels before INIT: hold after GardenToShoot1 settles, before shooting.
    public static double WAIT_AFTER_GARDEN_TO_SHOOT_1_SECONDS = 10.0;
    public static long LONG_PATH_TIMEOUT_MS = 8000;
    // Interrupt collection, waits and shooting with this much AUTO time left for parking.
    public static double PARK_RESERVE_SECONDS = 4.0;

    private boolean splitterDown;

    public RedCooperationDownAuto() {
        super(Alliance.RED);
    }

    @Override
    protected Pose startPose() {
        return redPose(Red.START);
    }

    @Override
    protected Command buildRoutine() {
        splitterDown = false;
        Pose start = startPose();
        Pose shoot1 = redPose(Red.SHOOT_1);
        Pose flowerPrep1 = redPose(Red.FLOWER_PREP_1);
        Pose flower1 = redPose(Red.FLOWER_1);
        Pose gardenPrep = redPose(Red.GARDEN_PREP);
        Pose garden = redPose(Red.GARDEN);
        Pose park = redPose(Red.STOP_1);

        Path start1ToShoot1 = line(start, shoot1);
        Path shoot1ToFlowerPrep1 = line(shoot1, flowerPrep1);
        Path flowerPrep1ToFlower1 = line(flowerPrep1, flower1,
                Interpolator.constant(flower1.heading()));
        Path flower1ToShoot1 = line(flower1, shoot1);
        Path shoot1ToGardenPrep = curve(shoot1,
                redPose(Red.SHOOT_1_TO_GARDEN_PREP_CTRL), gardenPrep,
                Interpolator.piecewise()
                        .until(0.5, Interpolator.linear(shoot1.heading(), gardenPrep.heading()))
                        .until(1.0, Interpolator.constant(gardenPrep.heading())));
        Path gardenPrepToGarden = line(gardenPrep, garden,
                Interpolator.constant(garden.heading()));
        Path gardenToShoot1 = line(garden, shoot1,
                Interpolator.piecewise()
                        .until(0.5, Interpolator.constant(garden.heading()))
                        .until(1.0, Interpolator.linear(garden.heading(), shoot1.heading())));
        Path shoot1ToStop1 = curve(shoot1, redPose(Red.SHOOT_1_TO_STOP_1_CTRL), park,
                Interpolator.piecewise()
                        .until(0.7, Interpolator.constant(shoot1.heading()))
                        .until(1.0, Interpolator.linear(shoot1.heading(), park.heading())));

        return new SequentialCommandGroup(
                untilSecondsLeft(new SequentialCommandGroup(
                        intakeSlow(),
                        spinUp(),
                        aimWhileShooting(shoot1,
                                follow(start1ToShoot1),
                                settleAt(shoot1),
                                shoot(SHOOT_SECONDS)),
                        follow(shoot1ToGardenPrep),
                        intakeOn(),
                        follow(gardenPrepToGarden),
                        waitSeconds(GARDEN_SECONDS),
                        intakeSlow(),
                        spinUp(),
                        aimWhileShooting(shoot1,
                                follow(gardenToShoot1, PATH_SPEED_FRACTION, LONG_PATH_TIMEOUT_MS),
                                settleAt(shoot1),
                                waitSeconds(WAIT_AFTER_GARDEN_TO_SHOOT_1_SECONDS),
                                shoot(SHOOT_SECONDS)),
                        lowerSplitter(),
                        follow(shoot1ToFlowerPrep1),
                        // As in Full Auto, never enter the FLOWER before deployment completes.
                        new WaitUntilCommand(() -> splitterDown && !intake.isSplitterMoving()),
                        intakeOn(),
                        follow(flowerPrep1ToFlower1),
                        waitSeconds(FLOWER_SECONDS),
                        intakeSlow(),
                        spinUp(),
                        aimWhileShooting(shoot1,
                                follow(flower1ToShoot1, PATH_SPEED_FRACTION, LONG_PATH_TIMEOUT_MS),
                                settleAt(shoot1),
                                shoot(SHOOT_SECONDS))), PARK_RESERVE_SECONDS),
                // The cutoff can interrupt a spin-up before shoot() gets to stop it.
                new InstantCommand(shooter::stop, shooter),
                intakeOff(),
                // Retract for parking, including an early cutoff during the FLOWER cycle.
                retractLoweredSplitter(),
                follow(shoot1ToStop1, PATH_SPEED_FRACTION, LONG_PATH_TIMEOUT_MS));
    }

    private Command lowerSplitter() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> !intake.isSplitterMoving()),
                new InstantCommand(() -> {
                    if (!splitterDown && intake.deploySplitter()) {
                        splitterDown = true;
                    }
                }, intake));
    }

    /** Retract for parking if the FLOWER cycle deployed the splitter. */
    private Command retractLoweredSplitter() {
        return new InstantCommand(() -> {
            if (splitterDown) {
                intake.stopSplitter();
                if (intake.retractSplitter()) {
                    splitterDown = false;
                }
            }
        }, intake);
    }

    private static Command waitSeconds(double seconds) {
        return new WaitCommand((long) (Math.max(0.0, seconds) * 1000));
    }
}
