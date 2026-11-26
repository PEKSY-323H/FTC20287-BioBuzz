package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
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
                Controller primaryTranslationalForward = Controller.proportional(0.264867697030224);
                Controller secondaryTranslationalForward = Controller.proportional(0.09786153665745687);
                Controller primaryTranslationalLateral = Controller.proportional(0.5503136735081704);
                Controller secondaryTranslationalLateral = Controller.proportional(0.20332619770909333);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01402256191176626));
                c.brake.set(Controller.proportionalFeedforward(0.011919177625001321));

                c.headingFeedback.set(Controller.proportional(4.137789836614438));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.049935590010808904, 0.00420192781143353));

                c.linearBrakeCoefficients.set(Matrix.diag(0.006998293018999701, 0.01129235034235655));
                c.quadraticBrakeCoefficients.set(Matrix.diag(4.9166986468482844E-5, -2.8424983050998285E-5));

                c.maxAchievableForwardVelocity.set(75.45876123355895);
                c.maxAchievableStrafeVelocity.set(62.26267301547017);
                c.naturalForwardDeceleration.set(40.82473725159998);
                c.naturalStrafeDeceleration.set(81.12802669435408);
            }
    );

    /**
     * Builds the follower; named {@code create} as in the Pedro 3 docs. It uses the OctoQuad
     * with the Control Hub IMU as heading backup. The OctoQuad starts recalibrating its IMU
     * here without blocking, so call this during INIT and keep the robot still until
     * {@link OctoQuadImuBackupLocalizer#getSource()} reports ODOMETRY.
     */
    public static Follower create(HardwareMap hardwareMap) {
        return create(hardwareMap, createBackupLocalizer(hardwareMap));
    }

    /** The follower around an already built localizer, so an OpMode can keep a handle to it. */
    public static Follower create(HardwareMap hardwareMap, Localizer localizer) {
        return new Follower(
                localizer,
                createDrivetrain(hardwareMap),
                new Foresight(foresightConfig));
    }

    /** OctoQuad localizer with Control Hub IMU heading backup; never blocks, never hangs INIT. */
    public static OctoQuadImuBackupLocalizer createBackupLocalizer(HardwareMap hardwareMap) {
        return new OctoQuadImuBackupLocalizer(hardwareMap, localizerConfig);
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
        c.xPodOffset.set(-3.484251968503937);
        c.yPodOffset.set(-6.338582677165355);
        c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.yPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        c.headingScalar.set(1.0079471955361563);
    });

    /**
     * The plain OctoQuad localizer, without the IMU backup: what pedro/Tuning hands to the
     * tuners, so tuning measures the OctoQuad alone. Blocks until the OctoQuad is RUNNING.
     */
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
