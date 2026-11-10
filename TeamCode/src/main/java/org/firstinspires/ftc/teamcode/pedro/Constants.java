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


    /**
     * TODO: every number here is Pedro's documentation sample, not tuned for this robot.
     * Replace the whole block with the output of the Foresight Tuner in Pedro AutoTune
     * (http://192.168.43.1:10158, registered in pedro/Tuning), after the localizer passes
     * the Odometry Test.
     */
//    public static ForesightConfig foresightConfig = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(2.0242194874368624);
//                Controller secondaryTranslationalForward = Controller.proportional(0.7478950124670608);
//                Controller primaryTranslationalLateral = Controller.proportional(0.3076221684840604);
//                Controller secondaryTranslationalLateral = Controller.proportional(0.11365817143913191);
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(0.015219045710296026));
//                c.brake.set(Controller.proportionalFeedforward(0.012936188853751621));
//
//                c.headingFeedback.set(Controller.proportional(3.906548351477625));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0036836358620452926, 0.012094332828020117));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(0.3578701207632757, 0.0717797633823807));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(-0.0044984662384110205, 0.0014173572025533727));
//
//                c.maxAchievableForwardVelocity.set(76.55414985135995);
//                c.maxAchievableStrafeVelocity.set(61.32722224017114);
//                c.naturalForwardDeceleration.set(28.473939750554397);
//                c.naturalStrafeDeceleration.set(56.77589090994222);
//            }
//    );
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                // Hand-tuned: AutoTune gave 4.696 / 1.735 here, about 16 times the strafe gains
                // below, and the robot shook in the Hold Test. 0.30 / 0.11 (equal to strafe)
                // stopped that but still shook front-to-back while approaching a target, so
                // forward now sits below strafe. Not yet re-tested on the robot.
                Controller primaryTranslationalForward = Controller.proportional(0.20);
                Controller secondaryTranslationalForward = Controller.proportional(0.07);
                Controller primaryTranslationalLateral = Controller.proportional(0.2862067047360195);
                Controller secondaryTranslationalLateral = Controller.proportional(0.10574572981596109);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.0204642745921879));
                c.brake.set(Controller.proportionalFeedforward(0.017394633403359713));

                c.headingFeedback.set(Controller.proportional(3.5491261388178916));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05954799785186256, 0.003778975715383811));

                // Predicted stopping distance = linear * v + quadratic * v * |v| (forward, strafe).
                // Linear terms hand-raised about 20 % from AutoTune's 0.2493 / 0.0653 so the
                // robot starts slowing earlier; it was driving past the target and coming
                // back. Not yet re-tested on the robot.
                c.linearBrakeCoefficients.set(Matrix.diag(0.30, 0.078));
                c.quadraticBrakeCoefficients.set(Matrix.diag(-0.0030620639876104147, 0.0014767345489988737));

                c.maxAchievableForwardVelocity.set(83.4973538997374);
                c.maxAchievableStrafeVelocity.set(66.749621026265);
                c.naturalForwardDeceleration.set(38.359098479327784);
                c.naturalStrafeDeceleration.set(68.29474265059757);
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
    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
        c.name.set("octoquad");
        c.xPodPort.set(1);
        c.yPodPort.set(0);
        c.ticksPerUnit.set(505.316944406);
        c.xPodOffset.set(4.980314960629921);
        c.yPodOffset.set(-1.062992125984252);
        c.xPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        c.headingScalar.set(1.0117874673938325);
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
