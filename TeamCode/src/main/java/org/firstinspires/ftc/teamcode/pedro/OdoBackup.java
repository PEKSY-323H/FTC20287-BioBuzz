package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Backup localizer for when the OctoQuad breaks: Pedro's Pinpoint localizer on a goBILDA
 * Pinpoint Odometry Computer, which uses its own IMU for heading. Nothing in the program uses
 * it; to switch, construct it where {@code Constants.create} builds the OctoQuad localizer.
 *
 * <p>Hardware: plug the forward (x) pod into the Pinpoint's X port and the strafe (y) pod
 * into its Y port, and add the Pinpoint to the robot configuration as "goBILDA Pinpoint
 * Odometry Computer" named {@code pinpoint}.
 *
 * <p>Same units and frame as the OctoQuad setup: inches and radians, +x forward, +y left,
 * heading CCW-positive. Like the OctoQuad, it recalibrates its IMU during INIT (Pedro waits
 * 500 ms), so keep the robot still.
 *
 * <p>Before a match on the backup, push the robot by hand in a TeleOp and watch the field
 * telemetry: forward must raise x, left must raise y, counterclockwise must raise heading.
 * If x or y moves the wrong way, flip that pod's direction below.
 */
@Disabled
public final class OdoBackup extends PinpointLocalizer {
    /**
     * Pod geometry copied from {@code Constants.localizerConfig}, which the OctoQuad Tuner
     * measured for these same pods: Pedro gives both devices the forward pod's sideways offset
     * (left positive) and the strafe pod's forward offset. 505.3 ticks/in is the goBILDA 4-bar
     * pod's 19.894 ticks/mm. The directions are unverified on the Pinpoint: run the push check.
     */
    public static final PinpointConfig CONFIG = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.xPodOffset.set(-3.484251968503937);
        c.yPodOffset.set(-6.338582677165355);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.resetMode.set(PinpointLocalizer.ResetMode.RECALIBRATE_IMU);
    });

    public OdoBackup(HardwareMap hardwareMap) {
        super(hardwareMap, CONFIG);
        checkPinpoint(hardwareMap.get(GoBildaPinpointDriver.class, CONFIG.name.get()));
    }

    /**
     * A Pinpoint with missing pods or a runaway IMU still reports a pose, just a wrong one.
     * Fail INIT with a readable message instead of driving on it.
     */
    private static void checkPinpoint(GoBildaPinpointDriver pinpoint) {
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        switch (status) {
            case FAULT_X_POD_NOT_DETECTED:
            case FAULT_Y_POD_NOT_DETECTED:
            case FAULT_NO_PODS_DETECTED:
                throw new IllegalStateException("Pinpoint reports " + status
                        + ": plug the forward pod into X and the strafe pod into Y.");
            case FAULT_IMU_RUNAWAY:
                throw new IllegalStateException("Pinpoint reports FAULT_IMU_RUNAWAY: its heading"
                        + " is unusable. Power-cycle the robot and re-INIT with it still.");
            default:
                break;
        }
    }
}
