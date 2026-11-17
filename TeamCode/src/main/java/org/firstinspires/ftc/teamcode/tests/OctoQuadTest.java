package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Sensor-only diagnostics: heading, calibration status, CRCs and all eight encoder counts. */
@TeleOp(name = "TEST - OctoQuad", group = "Testing")
public final class OctoQuadTest extends LinearOpMode {
    private static final String OCTOQUAD_NAME = "octoquad";
    private final OctoQuad.LocalizerDataBlock localizer = new OctoQuad.LocalizerDataBlock();
    private final OctoQuad.EncoderDataBlock encoders = new OctoQuad.EncoderDataBlock();
    private OctoQuad octoquad;
    private String firmware;
    private double displayZeroRadians;
    private long totalPackets;
    private long localizerCrcErrors;
    private long encoderCrcErrors;
    private boolean previousA;
    private boolean previousB;
    private String lastAction = "INIT calibration: keep the robot still.";

    @Override
    public void runOpMode() throws InterruptedException {
        octoquad = hardwareMap.get(OctoQuad.class, OCTOQUAD_NAME);
        if (octoquad.getChipId() != OctoQuad.OCTOQUAD_CHIP_ID) {
            throw new IllegalStateException("OctoQuad chip ID mismatch; check I2C wiring.");
        }
        OctoQuad.FirmwareVersion version = octoquad.getFirmwareVersion();
        firmware = version.toString();
        if (version.maj != OctoQuad.SUPPORTED_FW_VERSION_MAJ) {
            throw new IllegalStateException("Unsupported OctoQuad firmware " + firmware
                    + "; expected major " + OctoQuad.SUPPORTED_FW_VERSION_MAJ);
        }

        telemetry.setMsTransmissionInterval(100);
        // Reset only pose/calibration. Preserve pod settings, heading scalar and flash contents.
        octoquad.resetLocalizerAndCalibrateIMU();
        while (opModeInInit() && !isStopRequested()) {
            readAndReport(false);
            sleep(50);
        }

        waitForStart();
        if (isStopRequested()) {
            return;
        }
        previousA = gamepad1.a;
        previousB = gamepad1.b;
        while (opModeIsActive() && !isStopRequested()) {
            readAndReport(true);
            sleep(50);
        }
    }

    private void readAndReport(boolean allowButtons) {
        boolean currentA = gamepad1.a;
        boolean currentB = gamepad1.b;
        boolean zeroPressed = allowButtons && currentA && !previousA;
        boolean calibratePressed = allowButtons && currentB && !previousB;
        previousA = currentA;
        previousB = currentB;

        if (calibratePressed) {
            octoquad.resetLocalizerAndCalibrateIMU();
            displayZeroRadians = 0.0;
            lastAction = "B: recalibrating; keep the robot still.";
        }

        // One bulk transaction supplies both blocks; check their CRCs independently.
        octoquad.readLocalizerDataAndAllEncoderData(localizer, encoders);
        totalPackets++;
        if (!localizer.crcOk) {
            localizerCrcErrors++;
        }
        if (!encoders.crcOk) {
            encoderCrcErrors++;
        }
        boolean headingValid = localizer.isDataValid() && Float.isFinite(localizer.heading_rad);
        if (zeroPressed && !calibratePressed) {
            if (headingValid) {
                displayZeroRadians = localizer.heading_rad;
                lastAction = "A: display zero set; device pose unchanged.";
            } else {
                lastAction = "A ignored: wait for valid RUNNING heading.";
            }
        }

        telemetry.clearAll();
        telemetry.addData("Hardware name", OCTOQUAD_NAME);
        telemetry.addData("Firmware", firmware);
        if (allowButtons) {
            telemetry.addData("Phase", "RUNNING TEST");
        } else {
            telemetry.addData("Phase", "INIT: keep still for calibration");
        }
        if (localizer.crcOk) {
            telemetry.addData("Localizer status", localizer.localizerStatus);
            if (localizer.localizerStatus == OctoQuad.LocalizerStatus.FAULT_NO_IMU) {
                telemetry.addLine("No IMU detected: heading needs OctoQuad MK2.");
            }
        } else {
            telemetry.addData("Localizer status", "INVALID CRC: packet discarded");
        }
        telemetry.addData("Heading axis", octoquad.getLocalizerHeadingAxisChoice());
        if (headingValid) {
            double relativeRadians = localizer.heading_rad - displayZeroRadians;
            relativeRadians = Math.atan2(Math.sin(relativeRadians), Math.cos(relativeRadians));
            telemetry.addData("Heading deg", Math.toDegrees(localizer.heading_rad));
            telemetry.addData("Heading rad", localizer.heading_rad);
            telemetry.addData("Relative heading deg", Math.toDegrees(relativeRadians));
            if (Float.isFinite(localizer.velHeading_radS)) {
                telemetry.addData("Rotation deg/s", Math.toDegrees(localizer.velHeading_radS));
            } else {
                telemetry.addData("Rotation deg/s", "INVALID");
            }
        } else {
            telemetry.addData("Heading", "UNAVAILABLE: calibration/status/CRC");
        }

        if (encoders.crcOk) {
            for (int channel = 0; channel < OctoQuad.NUM_ENCODERS; channel++) {
                telemetry.addData("Encoder " + channel + " ticks", encoders.positions[channel]);
            }
        } else {
            telemetry.addData("Encoders", "INVALID CRC: packet discarded");
        }
        telemetry.addData("Localizer CRC errors / packets", localizerCrcErrors + " / " + totalPackets);
        telemetry.addData("Encoder CRC errors / packets", encoderCrcErrors + " / " + totalPackets);
        telemetry.addData("Last action", lastAction);
        telemetry.addLine("A: zero display | B: recalibrate while stationary");
        telemetry.addLine("Rotate robot CCW by hand: heading should increase.");
        telemetry.addLine("Turn each encoder by hand and watch its channel count.");
        telemetry.update();
    }
}
