package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

/**
 * Pedro Pathing 3 follower configuration: OctoQuad localizer + mecanum drivetrain + Foresight.
 *
 * <p>Units: inches and radians. Robot axes: +x forward, +y left, heading CCW-positive. The
 * field frame the team writes poses in is documented in {@code utils/Alliance}.
 *
 * <p>Every value marked TODO is a placeholder, NOT measured on this robot. Run the tuners
 * registered in {@code pedro/Tuning}, in the order given there, before driving any path.
 */
public final class Constants {
    private Constants() {
    }

    /**
     * Names and directions come straight from {@code subsystems/Drive}, the wiring the
     * TeleOps already drive with. Pedro uses the same rule: positive power on every wheel
     * rolls the robot forward.
     */
    public static final MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(Drive.FRONT_LEFT_NAME);
        c.frontRightName.set(Drive.FRONT_RIGHT_NAME);
        c.backLeftName.set(Drive.BACK_LEFT_NAME);
        c.backRightName.set(Drive.BACK_RIGHT_NAME);
        c.frontLeftDirection.set(Drive.FRONT_LEFT_DIRECTION);
        c.frontRightDirection.set(Drive.FRONT_RIGHT_DIRECTION);
        c.backLeftDirection.set(Drive.BACK_LEFT_DIRECTION);
        c.backRightDirection.set(Drive.BACK_RIGHT_DIRECTION);
        c.manualBrakeMode.set(true);
    });

    /**
     * Pedro writes these parameters to the OctoQuad's RAM on every INIT (not flash),
     * overriding whatever pod geometry the device had. The OctoQuad Tuner in Pedro AutoTune
     * (see pedro/Tuning) measures every TODO below and prints a replacement for this block;
     * TEST - OctoQuad shows which port each pod is on.
     */
    public static final OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
        c.name.set("octoquad");
        // TODO: confirm ports with TEST - OctoQuad (push forward → X pod changes).
        c.xPodPort.set(0);
        c.yPodPort.set(1);
        // TODO: flip so pushing forward raises x and pushing left raises y.
        c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        // TODO: measure each pod's offset from the rotation center (sign convention per the
        // Pedro OctoQuad docs). Check by turning in place: x/y must barely move; if they
        // trace a circle, flip the sign of the offending offset.
        c.offsetUnits.set(DistanceUnit.INCH);
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(0.0);
        // TODO: goBILDA 4-bar pod = 19.894 ticks/mm; swingarm pod = 13.263 ticks/mm.
        c.encoderResolutionUnit.set(DistanceUnit.MM);
        c.ticksPerUnit.set(19.89436789);
        // TODO: spin 10 turns, set to measuredDegrees / 3600.
        c.headingScalar.set(1.0);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
    });

    /**
     * TODO: every number here is Pedro's documentation sample, not tuned for this robot.
     * Replace the whole block with the output of the Foresight Tuner in Pedro AutoTune
     * (http://192.168.43.1:10158, registered in pedro/Tuning), after the localizer passes
     * the Odometry Test.
     */
    public static final ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryForward = Controller.proportional(0.3);
        Controller secondaryForward = Controller.proportional(0.1);
        Controller primaryLateral = Controller.proportional(0.3);
        Controller secondaryLateral = Controller.proportional(0.1);
        c.forwardTranslational.set(Controller.piecewise(secondaryForward).put(2.5, primaryForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryLateral).put(2.5, primaryLateral));
        c.coast.set(Controller.proportionalFeedforward(0.010978350889324107));
        c.brake.set(Controller.proportionalFeedforward(0.008731598255925491));
        c.headingFeedback.set(Controller.proportional(5.258721785960744));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695));
        c.linearBrakeCoefficients.set(Matrix.diag(0.10605894992901523, 0.08719146175596092));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014663966976606565, 0.0013837064502458813));
        c.maxAchievableForwardVelocity.set(72.72923108818539);
        c.maxAchievableStrafeVelocity.set(52.34323936525474);
        c.naturalForwardDeceleration.set(85.01144677379789);
        c.naturalStrafeDeceleration.set(104.49787535782846);
    });

    /**
     * Builds the follower; named {@code create} as in the Pedro 3 docs. The OctoQuad
     * localizer recalibrates its IMU and blocks until the device reports RUNNING, so call
     * this during INIT with the robot still.
     */
    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                createLocalizer(hardwareMap),
                createDrivetrain(hardwareMap),
                new Foresight(foresightConfig));
    }

    /** The configured localizer on its own; also what pedro/Tuning hands to the tuners. */
    public static OctoQuadLocalizer createLocalizer(HardwareMap hardwareMap) {
        checkOctoQuad(hardwareMap);
        return new OctoQuadLocalizer(hardwareMap, localizerConfig);
    }

    /** The configured drivetrain on its own; also what pedro/Tuning hands to the tuners. */
    public static Mecanum createDrivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    /**
     * Pedro's OctoQuadLocalizer waits for RUNNING with no timeout, so a miswired or
     * IMU-less OctoQuad would hang INIT forever. Fail with a readable message instead.
     */
    private static void checkOctoQuad(HardwareMap hardwareMap) {
        OctoQuad octoQuad = hardwareMap.get(OctoQuad.class, localizerConfig.name.get());
        if (octoQuad.getChipId() != OctoQuad.OCTOQUAD_CHIP_ID) {
            throw new IllegalStateException("OctoQuad chip ID mismatch: check the I2C wiring"
                    + " and that the device is configured as OctoQuadFTC.");
        }
        if (octoQuad.getLocalizerStatus() == OctoQuad.LocalizerStatus.FAULT_NO_IMU) {
            throw new IllegalStateException("OctoQuad reports no IMU: Pedro's OctoQuad"
                    + " localizer needs an OctoQuad MK2.");
        }
    }
}
