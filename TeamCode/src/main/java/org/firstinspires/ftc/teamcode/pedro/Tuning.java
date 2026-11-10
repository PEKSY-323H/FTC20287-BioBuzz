package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.OctoQuadTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * Registers Pedro Pathing's AutoTune procedures for this robot (mecanum + OctoQuad).
 *
 * <p>Deploy, connect to the Robot Controller's Wi-Fi and open http://192.168.43.1:10158.
 * Run them in this order, pasting each generated block over the matching one in
 * {@link Constants} and redeploying before the next step:
 * <ol>
 * <li>Mecanum Tuner: wheels off the ground. It only confirms motor names and directions,
 *     which must end up equal to the constants in {@code subsystems/Drive}; if a wheel is
 *     wrong, fix it there, not in Constants.</li>
 * <li>OctoQuad Tuner: pushed by hand, no motor power. Produces {@code localizerConfig}.</li>
 * <li>Tests, Odometry Test: the robot drives itself forward, left and counterclockwise.
 *     Do not go on until it passes.</li>
 * <li>Foresight Tuner: the robot drives and spins at full power and coasts past its
 *     target, so use a clear floor with a person on the Driver Station STOP. Produces
 *     {@code foresightConfig}.</li>
 * <li>Tests, Hold / Line / Curve: checks the finished follower.</li>
 * </ol>
 *
 * <p>The files in {@code procedures/} are unmodified copies from the official Quickstart,
 * github.com/Pedro-Pathing/Quickstart at commit 9dc89787898fab6b5cd9095df38489a14bf51a95
 * (2026-10-23), limited to the four this robot needs. Re-copy them rather than editing.
 */
public final class Tuning {
    private Tuning() {
    }

    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure octoquadTuner() {
        return new OctoQuadTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(Constants::createLocalizer, Constants::createDrivetrain);
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(Constants::createDrivetrain, Constants::createLocalizer,
                () -> new Foresight(Constants.foresightConfig));
    }
}
