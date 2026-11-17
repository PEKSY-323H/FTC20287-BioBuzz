package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autos.Points.Blue;
import org.firstinspires.ftc.teamcode.utils.Alliance;

import java.util.Collections;

/** Shoots the preload once, collects FLOWER 1, then carries the pollen directly to STOP_1. */
@Configurable
@Autonomous(name = "Blue Cooperation Down Park Auto", group = "Blue", preselectTeleOp = "Field-Centric")
public final class BlueCooperationDownParkAuto extends BaseAuto {
    public static double SHOOT_SECONDS = 2.0;
    public static double FLOWER_SECONDS = 2.0;
    public static double PARK_RESERVE_SECONDS = 4.0;
    public static long LONG_PATH_TIMEOUT_MS = 8000;

    private boolean splitterDown;

    public BlueCooperationDownParkAuto() {
        super(Alliance.BLUE);
    }

    @Override
    protected Pose startPose() {
        return bluePose(Blue.START);
    }

    @Override
    protected Command buildRoutine() {
        splitterDown = false;
        Pose shoot1 = bluePose(Blue.SHOOT_1);
        Pose flowerPrep1 = bluePose(Blue.FLOWER_PREP_1);
        Pose flower1 = bluePose(Blue.FLOWER_1);
        Pose park = bluePose(Blue.STOP_1);
        Path startToShoot1 = line(startPose(), shoot1, Interpolator.constant(shoot1.heading()));
        Path shoot1ToFlowerPrep1 = line(shoot1, flowerPrep1);
        Path flowerPrep1ToFlower1 = line(flowerPrep1, flower1,
                Interpolator.constant(flower1.heading()));

        return new SequentialCommandGroup(
                untilSecondsLeft(new SequentialCommandGroup(
                        intakeSlow(),
                        spinUp(),
                        follow(startToShoot1),
                        settleAt(shoot1),
                        shoot(SHOOT_SECONDS),
                        new WaitUntilCommand(() -> !intake.isSplitterMoving()),
                        new InstantCommand(() -> {
                            if (intake.deploySplitter()) {
                                splitterDown = true;
                            }
                        }, intake),
                        follow(shoot1ToFlowerPrep1),
                        new WaitUntilCommand(() -> splitterDown && !intake.isSplitterMoving()),
                        intakeOn(),
                        follow(flowerPrep1ToFlower1),
                        new WaitCommand((long) (Math.max(0.0, FLOWER_SECONDS) * 1000))),
                        PARK_RESERVE_SECONDS),
                new InstantCommand(shooter::stop, shooter),
                intakeSlow(),
                new InstantCommand(() -> {
                    if (splitterDown) {
                        intake.stopSplitter();
                        if (intake.retractSplitter()) {
                            splitterDown = false;
                        }
                    }
                }, intake),
                // Build from the actual pose so an early cutoff can also go directly to STOP_1.
                new DeferredCommand(() -> follow(line(follower.getPose(), park),
                        PATH_SPEED_FRACTION, LONG_PATH_TIMEOUT_MS), Collections.singletonList(follower)),
                intakeOff()) {
            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                intake.stop();
                shooter.stop();
            }
        };
    }
}
