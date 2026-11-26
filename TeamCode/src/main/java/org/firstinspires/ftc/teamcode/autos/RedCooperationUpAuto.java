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
 * RedCooperateUp.pp.json route using the shared, tuned RED waypoints.
 * Shoots the preload at SHOOT_2, collects FLOWER 2, returns to SHOOT_2 to shoot, then
 * parks at STOP_2. Deploys the splitter on the FLOWER approach and retracts when parking
 * starts. Shooting uses BaseAuto's TeleOp-style gate/readiness check.
 */
@Configurable
@Autonomous(name = "Red Cooperation Up Auto", group = "Red", preselectTeleOp = "Field-Centric")
public final class RedCooperationUpAuto extends BaseAuto {
    public static double SHOOT_SECONDS = 2.0;
    public static double FLOWER_SECONDS = 2.0;
    // Set in Panels before INIT: wait at SHOOT_2 after arriving from START_2.
    public static double WAIT_BEFORE_FIRST_SHOOT_SECONDS = 6;
    // Wait before the second shot, after returning from FLOWER_2.
    public static double WAIT_BEFORE_SHOOT_SECONDS = 1.5;
    public static double PARK_RESERVE_SECONDS = 4.0;
    public static long LONG_PATH_TIMEOUT_MS = 8000;

    private boolean splitterDown;

    public RedCooperationUpAuto() {
        super(Alliance.RED);
    }

    @Override
    protected Pose startPose() {
        return redPose(Red.START_2);
    }

    @Override
    protected Command buildRoutine() {
        splitterDown = false;
        Pose start = startPose();
        Pose shoot2 = redPose(Red.SHOOT_2);
        Pose flowerPrep2 = redPose(Red.FLOWER_PREP_2);
        Pose flower2 = redPose(Red.FLOWER_2);
        Pose park = redPose(Red.STOP_2);

        Path start2ToShoot2 = curve(start, redPose(Red.START_2_TO_SHOOT_2_CTRL), shoot2);
        Path shoot2ToFlowerPrep2 = line(shoot2, flowerPrep2);
        Path flowerPrep2ToFlower2 = line(flowerPrep2, flower2,
                Interpolator.constant(flower2.heading()));
        Path flower2ToShoot2 = curve(flower2, redPose(Red.FLOWER_2_TO_SHOOT_2_CTRL), shoot2);
        Path shoot2ToStop2 = line(shoot2, park,
                Interpolator.piecewise()
                        .until(0.5, Interpolator.linear(shoot2.heading(), park.heading()))
                        .until(1.0, Interpolator.constant(park.heading())));

        return new SequentialCommandGroup(
                untilSecondsLeft(new SequentialCommandGroup(
                        intakeSlow(),
                        spinUp(),
                        aimWhileShooting(shoot2,
                                follow(start2ToShoot2, PATH_SPEED_FRACTION, LONG_PATH_TIMEOUT_MS),
                                settleAt(shoot2),
                                waitSeconds(WAIT_BEFORE_FIRST_SHOOT_SECONDS),
                                shoot(SHOOT_SECONDS)),
                        lowerSplitter(),
                        follow(shoot2ToFlowerPrep2),
                        new WaitUntilCommand(() -> splitterDown && !intake.isSplitterMoving()),
                        intakeOn(),
                        follow(flowerPrep2ToFlower2),
                        waitSeconds(FLOWER_SECONDS),
                        intakeSlow(),
                        spinUp(),
                        aimWhileShooting(shoot2,
                                follow(flower2ToShoot2, PATH_SPEED_FRACTION, LONG_PATH_TIMEOUT_MS),
                                settleAt(shoot2),
                                waitSeconds(WAIT_BEFORE_SHOOT_SECONDS),
                                shoot(SHOOT_SECONDS))), PARK_RESERVE_SECONDS),
                new InstantCommand(shooter::stop, shooter),
                intakeOff(),
                new InstantCommand(() -> {
                    if (splitterDown) {
                        intake.stopSplitter();
                        if (intake.retractSplitter()) {
                            splitterDown = false;
                        }
                    }
                }, intake),
                follow(shoot2ToStop2, PATH_SPEED_FRACTION, LONG_PATH_TIMEOUT_MS));
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

    private static Command waitSeconds(double seconds) {
        return new WaitCommand((long) (Math.max(0.0, seconds) * 1000));
    }
}
