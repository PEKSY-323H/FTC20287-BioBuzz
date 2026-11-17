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
 * <p>Current configuration blocks are below. Verify them on this robot using the tuners
 * registered in {@code pedro/Tuning}, in the order given there, before relying on paths.
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

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                // Conservative starting limits: the rear wheels lifted during the line test.
                // Reduce approach speed and opposing motor power before retesting braking.
                c.maxVelocityConstraint.set(70.0);     // inches/s
                c.maxBrakingPower.set(0.1);

                Controller primaryTranslationalForward = Controller.proportional(0.23372204768170443);
                Controller secondaryTranslationalForward = Controller.proportional(0.08635405145025682);
                Controller primaryTranslationalLateral = Controller.proportional(0.36505318684010185);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1348773981366159);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01129890090250132));
                c.brake.set(Controller.proportionalFeedforward(0.009604065767126123));

                c.headingFeedback.set(Controller.proportional(3.2878116355549833));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06719381319638153, 0.003161863919029447));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09682320833335036, 0.04511120840579613));
                c.quadraticBrakeCoefficients.set(Matrix.diag(3.065463350559885E-4, 0.00170514316641828));

                c.maxAchievableForwardVelocity.set(82.06769753057878);
                c.maxAchievableStrafeVelocity.set(69.95459700382189);
                c.naturalForwardDeceleration.set(55.55486577157712);
                c.naturalStrafeDeceleration.set(78.47823301503256);
            }
    );

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

    /**
     * Pedro writes these parameters to the OctoQuad's RAM on every INIT (not flash),
     * overriding whatever pod geometry the device had. The OctoQuad Tuner in Pedro AutoTune
     * (see pedro/Tuning) prints a replacement localizerConfig block;
     * TEST - OctoQuad shows which port each pod is on.
     */
    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
        c.name.set("octoquad");
        c.xPodPort.set(1);
        c.yPodPort.set(0);
        c.ticksPerUnit.set(505.316944406);
        c.xPodOffset.set(4.094488188976378);
        c.yPodOffset.set(-1.988188976377953);
        c.xPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        c.headingScalar.set(1.012570131791675);
    });

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
