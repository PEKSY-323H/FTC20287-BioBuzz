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

import org.firstinspires.ftc.teamcode.utils.Alliance;

/**
 * Eight segments from Auto routines/RedFull.pp.json, in sequence. Coordinates and headings
 * are in the team RED field frame. The JSON contains no mechanism markers or waits, so
 * shooting and collection actions are attached to its named stops here.
 */
@Configurable
@Autonomous(name = "Red Full Auto", group = "Red", preselectTeleOp = "Field-Centric")
public final class RedFullAuto extends BaseAuto {
    public static double SHOOT_SECONDS = 2.0;
    public static double FLOWER_SECONDS = 0.5;
    public static long LONG_PATH_TIMEOUT_MS = 8000;

    public RedFullAuto() {
        super(Alliance.RED);
    }

    @Override
    protected Pose startPose() {
        return redPose(57.67597307477678, 6.943703787667412, 90);
    }

    @Override
    protected Command buildRoutine() {
        Pose start = startPose();
        Pose shoot1 = redPose(57.67473929268973, 15.784066336495535, -90);
        Pose pollenPrep1 = redPose(27.75, 46.75, 180);
        Pose pollen1 = redPose(11.75, 46.75, 180);
        Pose shoot2 = redPose(57.943067278180806, 126.61983598981584, 0);
        Pose pollenPrep2 = redPose(46.838215419224326, 116.75, 90);
        Pose pollen2 = redPose(46.75, 129.75, 90);
        Pose shoot2Return = redPose(58.05410766601562, 126.57603672572543, 90);
        Pose stop = redPose(9.43345642089844, 96.21348135811941, 0);

        Path startToShoot1 = line(start, shoot1, Interpolator.constant(Math.toRadians(-90)));
        Path shoot1ToPollenPrep1 = line(shoot1, pollenPrep1,
                Interpolator.linear(Math.toRadians(-90), Math.toRadians(180)));
        Path pollenPrep1ToPollen1 = line(pollenPrep1, pollen1,
                Interpolator.constant(Math.toRadians(180)));
        Path pollen1ToShoot2 = curve(pollen1,
                redPose(27.358844370268475, 109.98184479179977, 0), shoot2,
                Interpolator.piecewise()
                        .until(0.3,
                                Interpolator.linear(Math.toRadians(180), Math.toRadians(90)))
                        .until(1.0, Interpolator.constant(0)));
        Path shoot2ToPollenPrep2 = line(shoot2, pollenPrep2,
                Interpolator.constant(Math.toRadians(90)));
        Path pollenPrep2ToPollen2 = line(pollenPrep2, pollen2, Interpolator.tangent);
        Path pollen2ToShoot2 = curve(pollen2,
                redPose(48.23847961425781, 107.75913565499441, 0), shoot2Return,
                Interpolator.constant(Math.toRadians(90)));
        Path shoot2ToStop = curve(shoot2Return,
                redPose(50.199837427294796, 94.67747926044643, 0), stop,
                Interpolator.piecewise()
                        .until(0.3,
                                Interpolator.linear(Math.toRadians(90), 0))
                        .until(1.0, Interpolator.constant(0)));

        return new SequentialCommandGroup(
                spinUp(),
                follow(startToShoot1),
                shoot(SHOOT_SECONDS),
                follow(shoot1ToPollenPrep1),
                collectAtFlower(pollenPrep1ToPollen1),
                spinUp(),
                follow(pollen1ToShoot2, PATH_SPEED_FRACTION, LONG_PATH_TIMEOUT_MS),
                shoot(SHOOT_SECONDS),
                follow(shoot2ToPollenPrep2),
                collectAtFlower(pollenPrep2ToPollen2),
                spinUp(),
                follow(pollen2ToShoot2),
                shoot(SHOOT_SECONDS),
                follow(shoot2ToStop, PATH_SPEED_FRACTION, LONG_PATH_TIMEOUT_MS)) {
            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                shooter.stop();
                intake.stop();
                intake.stopSplitter();
            }
        };
    }

    private Command collectAtFlower(Path path) {
        return new SequentialCommandGroup(
                intakeOn(),
                follow(path),
                new InstantCommand(intake::deploySplitter, intake),
                new WaitCommand((long) (FLOWER_SECONDS * 1000))) {
            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                intake.stop();
                intake.stopSplitter();
            }
        };
    }
}
