package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autos.Points.Blue;
import org.firstinspires.ftc.teamcode.utils.Alliance;

/** Starts at START (start1), shoots the preload once at SHOOT_1, then parks at STOP_1. */
@Configurable
@Autonomous(name = "Blue Shoot 1 Auto", group = "Blue", preselectTeleOp = "Field-Centric")
public final class BlueShoot1Auto extends BaseAuto {
    public static double SHOOT_SECONDS = 3.0;
    public static double PARK_RESERVE_SECONDS = 4.0;
    public static long PARK_PATH_TIMEOUT_MS = 8000;

    public BlueShoot1Auto() {
        super(Alliance.BLUE);
    }

    @Override
    protected Pose startPose() {
        return bluePose(Blue.START);
    }

    @Override
    protected Command buildRoutine() {
        Pose start = startPose();
        Pose shoot1 = bluePose(Blue.SHOOT_1);
        Pose park = bluePose(Blue.STOP_1);
        Path startToShoot1 = line(start, shoot1);
        Path shoot1ToStop1 = curve(shoot1, bluePose(Blue.SHOOT_1_TO_STOP_1_CTRL), park,
                Interpolator.piecewise()
                        .until(0.7, Interpolator.constant(shoot1.heading()))
                        .until(1.0, Interpolator.linear(shoot1.heading(), park.heading())));

        return new SequentialCommandGroup(
                untilSecondsLeft(new SequentialCommandGroup(
                        intakeSlow(),
                        spinUp(),
                        aimWhileShooting(shoot1,
                                follow(startToShoot1),
                                settleAt(shoot1),
                                shoot(SHOOT_SECONDS))), PARK_RESERVE_SECONDS),
                // Also stop a spin-up interrupted before shoot() begins.
                new InstantCommand(shooter::stop, shooter),
                intakeOff(),
                retractSplitter(),
                follow(shoot1ToStop1, PATH_SPEED_FRACTION, PARK_PATH_TIMEOUT_MS));
    }
}
