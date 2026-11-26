package org.firstinspires.ftc.teamcode.pedro;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Locale;
import java.util.Map;

/**
 * Pedro localizer: the OctoQuad (odometry computer) is the primary source, and the Control
 * Hub IMU takes over the heading whenever the OctoQuad stops delivering valid data.
 *
 * <p>Nothing in here waits: INIT starts the OctoQuad calibration and returns, and every
 * {@link #update()} does at most one OctoQuad read and one IMU read.
 *
 * <ul>
 * <li>OctoQuad healthy: the pose is exactly what Pedro's OctoQuadLocalizer reports. The IMU is
 *     read every {@link #IMU_SAMPLE_PERIOD_MS} to keep an offset to the OctoQuad heading and
 *     to check that the IMU really turns with the robot.</li>
 * <li>A short dropout (under {@link #GLITCH_HOLD_MS}, OctoQuad still RUNNING): the last
 *     OctoQuad state is held, which is what Pedro's own localizer does.</li>
 * <li>A longer dropout: the heading comes from the IMU. The IMU cannot measure translation, so
 *     x and y stay at the last good position and the translational velocity is zero;
 *     {@link #isPositionTracked()} turns false.</li>
 * <li>The OctoQuad restarted (brownout, cable): it loses the parameters Pedro writes to its RAM.
 *     They are written again, the OctoQuad recalibrates, and the current estimate is copied
 *     into it before it is trusted again.</li>
 * </ul>
 *
 * <p>Units and frame are Pedro's (inches, radians, CCW-positive heading). Tuning still uses
 * the plain OctoQuad localizer from {@link Constants#createLocalizer}.
 */
@Configurable
public final class OctoQuadImuBackupLocalizer implements Localizer {
    // ---------- Control Hub IMU (read at INIT) ----------
    public static String IMU_NAME = "imu";
    // Only the logo's UP/DOWN choice affects yaw for a flat-mounted hub; the USB direction
    // only shifts yaw by a constant, which the sync removes. A wrong setting is caught by the
    // turn check and shown as "IMU MISMATCH" in telemetry.
    public static RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // ---------- Live tuning ----------
    // IMU turn multiplier, like the OctoQuad's headingScalar. 1.0 until measured.
    public static double IMU_YAW_SCALAR = 1.0;
    // How often the IMU is read while the OctoQuad is healthy. Every loop once it is not.
    public static long IMU_SAMPLE_PERIOD_MS = 50;
    // The IMU offset is only re-synced while turning slower than this, so the few ms between
    // the OctoQuad read and the IMU read cannot skew it.
    public static double SYNC_MAX_TURN_RATE_RAD_S = 1.5;
    // A dropout shorter than this holds the last OctoQuad state instead of switching to the IMU.
    public static long GLITCH_HOLD_MS = 100;
    // Consecutive valid OctoQuad frames needed before leaving the IMU backup.
    public static int RECOVERY_FRAMES = 3;
    // On rejoining, an OctoQuad heading this far from a verified IMU means it restarted unseen.
    public static double REJOIN_HEADING_TOLERANCE_DEG = 25.0;
    // The IMU is judged every time the robot has turned this far with a healthy OctoQuad.
    public static double IMU_CHECK_WINDOW_DEG = 90.0;
    // ... and rejected if its turn differs from the OctoQuad's by more than this fraction.
    public static double IMU_CHECK_MAX_ERROR_RATIO = 0.2;
    // Use the IMU before the robot has turned enough to verify it (e.g. at the start of AUTO).
    public static boolean USE_UNVERIFIED_IMU = true;
    // A restart command is sent again if the OctoQuad still reports NOT_INITIALIZED this long after it.
    public static long RESTART_RETRY_MS = 1000;

    /** Where the current pose comes from. */
    public enum Source {
        /** OctoQuad data, as Pedro's OctoQuadLocalizer would report it. */
        ODOMETRY,
        /** Short dropout: the last OctoQuad state, held. */
        HOLD_LAST,
        /** OctoQuad unavailable: IMU heading, x/y frozen at the last good position. */
        IMU_HEADING,
        /** Neither sensor usable: the whole pose is frozen. */
        FROZEN
    }

    /** Result of comparing the IMU's turning with the OctoQuad's. */
    public enum ImuCheck { UNVERIFIED, VERIFIED, MISMATCH }

    private final OctoQuadConfig config;
    private final DistanceUnit unit;
    private final OctoQuad octoQuad;
    private final IMU imu;
    private final OctoQuad.LocalizerDataBlock block = new OctoQuad.LocalizerDataBlock();

    private MotionState state = MotionState.ofVelocity(new Pose(0, 0, 0), Velocity.zero());
    private MotionState lastGood = state;
    private Source source = Source.FROZEN;
    private String octoQuadProblem;
    private String imuProblem;

    // OctoQuad bookkeeping.
    private boolean restartPending;
    private long restartRequestNs;
    private boolean octoQuadPoseStale = true;
    private boolean octoQuadInterrupted = true;
    private int goodFrames;
    private long faultStartNs;
    private int restartCount;

    // IMU bookkeeping: heading = syncHeading + IMU_YAW_SCALAR * imuTurnSinceSync.
    private boolean imuReadOk;
    private boolean imuSampled;
    private boolean haveYaw;
    private double lastYaw;
    private long lastImuReadNs;
    private double syncHeading;
    private double imuTurnSinceSync;
    private boolean synced = true;
    private ImuCheck imuCheck = ImuCheck.UNVERIFIED;
    private boolean checkHavePrevious;
    private double checkPreviousOctoHeading;
    private double checkPreviousYaw;
    private double checkTurned;
    private double checkError;
    private double previousBackupHeading = Double.NaN;
    private long previousBackupNs;

    public OctoQuadImuBackupLocalizer(HardwareMap hardwareMap, OctoQuadConfig config) {
        this.config = config;
        unit = config.globalDistanceUnit.get();

        octoQuad = hardwareMap.tryGet(OctoQuad.class, config.name.get());
        if (octoQuad == null) {
            reportOctoQuad("'" + config.name.get() + "' is not in the robot configuration");
        } else {
            if (octoQuad.getChipId() != OctoQuad.OCTOQUAD_CHIP_ID) {
                reportOctoQuad("chip ID mismatch: check the I2C wiring and that it is configured as OctoQuadFTC");
            }
            restartOctoQuad();
            // The INIT calibration is not a mid-run restart.
            restartCount = 0;
        }

        imu = initializeImu(hardwareMap);
        lastImuReadNs = System.nanoTime();
        if (imu != null) {
            readImu(lastImuReadNs);
        }
    }

    private IMU initializeImu(HardwareMap hardwareMap) {
        IMU found = hardwareMap.tryGet(IMU.class, IMU_NAME);
        if (found == null) {
            reportImu("no IMU named '" + IMU_NAME + "' in the robot configuration");
            return null;
        }
        RevHubOrientationOnRobot orientation;
        try {
            orientation = new RevHubOrientationOnRobot(IMU_LOGO_DIRECTION, IMU_USB_DIRECTION);
        } catch (IllegalArgumentException e) {
            // The OctoQuad does not need the backup, so a bad setting disables only the backup.
            reportImu("IMU_LOGO_DIRECTION / IMU_USB_DIRECTION: " + e.getMessage());
            return null;
        }
        if (!found.initialize(new IMU.Parameters(orientation))) {
            reportImu("initialize() failed");
            return null;
        }
        return found;
    }

    // ---------- Localizer ----------

    @Override
    public void update() {
        long now = System.nanoTime();
        boolean frameValid = readOctoQuad(now);

        imuSampled = false;
        if (imu != null && (!frameValid || source != Source.ODOMETRY
                || now - lastImuReadNs >= IMU_SAMPLE_PERIOD_MS * 1_000_000L)) {
            readImu(now);
        }

        if (frameValid) {
            onValidFrame(now);
        } else {
            onInvalidFrame(now);
        }
    }

    @Override
    public void setPose(Pose pose) {
        state = state.withPose(pose);
        lastGood = state;
        syncHeading = pose.heading();
        imuTurnSinceSync = 0.0;
        synced = true;
        checkHavePrevious = false;
        previousBackupHeading = Double.NaN;
        if (octoQuad == null) {
            return;
        }
        if (source == Source.ODOMETRY && !restartPending) {
            writeOctoQuadPose(pose);
        } else {
            // Copied in once the OctoQuad delivers valid data again.
            octoQuadPoseStale = true;
        }
    }

    @Override
    public MotionState state() {
        return state;
    }

    /** Restarts the OctoQuad localizer and IMU calibration without waiting for it to finish. */
    @Override
    public void reset() {
        if (octoQuad != null) {
            restartOctoQuad();
        }
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> map = Localizer.super.debug();
        map.put("source", source);
        map.put("octoQuad", octoQuadProblem == null ? "OK" : octoQuadProblem);
        map.put("imu", imuProblem == null ? imuCheck : imuProblem);
        return map;
    }

    // ---------- Status for OpModes ----------

    public Source getSource() {
        return source;
    }

    /** False while x/y are frozen (IMU backup or no sensor): don't start position-based paths. */
    public boolean isPositionTracked() {
        return source == Source.ODOMETRY || source == Source.HOLD_LAST;
    }

    /** One telemetry line: source, OctoQuad state, IMU state. */
    public String statusLine() {
        String sourceText;
        switch (source) {
            case ODOMETRY:
                sourceText = "OctoQuad";
                break;
            case HOLD_LAST:
                sourceText = "OctoQuad (holding through dropout)";
                break;
            case IMU_HEADING:
                sourceText = "IMU BACKUP: heading only, x/y frozen";
                break;
            default:
                sourceText = "NO SENSOR: pose frozen";
                break;
        }
        String imuText = imuProblem != null ? "IMU " + imuProblem : "IMU " + imuCheck;
        String octoText = octoQuadProblem == null ? "OctoQuad OK" : "OctoQuad " + octoQuadProblem;
        return String.format(Locale.US, "%s | %s | %s | restarts %d",
                sourceText, octoText, imuText, restartCount);
    }

    // ---------- OctoQuad ----------

    /** Reads one frame. True only for a valid frame that already reflects our pose. */
    private boolean readOctoQuad(long now) {
        if (octoQuad == null) {
            return false;
        }
        try {
            octoQuad.readLocalizerData(block);
        } catch (RuntimeException e) {
            // A disconnected device can return a short buffer; degrade to the backup.
            reportOctoQuad("read failed: " + e);
            return false;
        }
        if (!block.crcOk) {
            reportOctoQuad("CRC error (I2C dropout)");
            return false;
        }
        OctoQuad.LocalizerStatus status = block.localizerStatus;
        if (status != OctoQuad.LocalizerStatus.RUNNING) {
            octoQuadInterrupted = true;
            octoQuadPoseStale = true;
            reportOctoQuad(status.toString());
            boolean restarted = status == OctoQuad.LocalizerStatus.NOT_INITIALIZED
                    || status == OctoQuad.LocalizerStatus.WARMING_UP_IMU
                    || status == OctoQuad.LocalizerStatus.CALIBRATING_IMU;
            boolean retryDue = status == OctoQuad.LocalizerStatus.NOT_INITIALIZED
                    && now - restartRequestNs >= RESTART_RETRY_MS * 1_000_000L;
            if (restarted && (!restartPending || retryDue)) {
                // Not started by us (or our command was lost): it restarted with its RAM cleared.
                restartOctoQuad();
            }
            return false;
        }
        restartPending = false;
        if (!Float.isFinite(block.heading_rad) || !Float.isFinite(block.velHeading_radS)) {
            reportOctoQuad("non-finite data");
            return false;
        }
        if (octoQuadPoseStale) {
            // This frame predates our pose; the next ones will contain it.
            writeOctoQuadPose(state.pose());
            octoQuadPoseStale = false;
            return false;
        }
        octoQuadProblem = null;
        return true;
    }

    /** Writes Pedro's parameters again and starts a calibration; RUNNING is polled in update(). */
    private void restartOctoQuad() {
        restartPending = true;
        restartRequestNs = System.nanoTime();
        octoQuadPoseStale = true;
        octoQuadInterrupted = true;
        restartCount++;
        try {
            // Same parameters, in the same order, as Pedro's OctoQuadLocalizer constructor.
            octoQuad.setSingleEncoderDirection(config.xPodPort.get(), config.xPodDirection.get());
            octoQuad.setSingleEncoderDirection(config.yPodPort.get(), config.yPodDirection.get());
            double mmPerUnit = config.encoderResolutionUnit.get().toMm(1.0);
            float ticksPerMm = (float) (config.ticksPerUnit.get() / mmPerUnit);
            octoQuad.setAllLocalizerParameters(
                    config.xPodPort.get(),
                    config.yPodPort.get(),
                    ticksPerMm,
                    ticksPerMm,
                    (float) -config.offsetUnits.get().toMm(config.yPodOffset.get()),
                    (float) -config.offsetUnits.get().toMm(config.xPodOffset.get()),
                    config.headingScalar.get().floatValue(),
                    config.localizerVelocityIntervalMS.get());
            octoQuad.setI2cRecoveryMode(config.i2cRecoveryMode.get());
            octoQuad.resetLocalizerAndCalibrateIMU();
        } catch (RuntimeException e) {
            reportOctoQuad("restart failed: " + e);
        }
    }

    private void writeOctoQuadPose(Pose pose) {
        try {
            octoQuad.setLocalizerPose((int) unit.toMm(pose.x()), (int) unit.toMm(pose.y()),
                    (float) pose.heading());
        } catch (RuntimeException e) {
            reportOctoQuad("pose write failed: " + e);
            octoQuadPoseStale = true;
        }
    }

    private void onValidFrame(long now) {
        Pose pose = new Pose(unit.fromMm(block.posX_mm), unit.fromMm(block.posY_mm), block.heading_rad);
        Velocity velocity = new Velocity(unit.fromMm(block.velX_mmS), unit.fromMm(block.velY_mmS),
                block.velHeading_radS);
        MotionState frame = MotionState.ofVelocity(pose, velocity);
        goodFrames++;

        boolean rejoin = source == Source.ODOMETRY
                || (source == Source.HOLD_LAST && !octoQuadInterrupted)
                || goodFrames >= Math.max(1, RECOVERY_FRAMES);
        if (!rejoin) {
            useBackup(now);
            return;
        }
        if (source == Source.IMU_HEADING && imuCheck == ImuCheck.VERIFIED && imuReadOk) {
            double mismatch = Math.abs(AngleUnit.normalizeRadians(pose.heading() - state.pose().heading()));
            if (mismatch > Math.toRadians(REJOIN_HEADING_TOLERANCE_DEG)) {
                reportOctoQuad(String.format(Locale.US,
                        "heading off by %.0f deg after dropout; restarting it", Math.toDegrees(mismatch)));
                restartOctoQuad();
                goodFrames = 0;
                useBackup(now);
                return;
            }
        }

        if (source != Source.ODOMETRY) {
            // The IMU turn check must not span the dropout.
            checkHavePrevious = false;
        }
        if (source == Source.IMU_HEADING || source == Source.FROZEN) {
            // Realign the IMU to the OctoQuad now, or at the next IMU sample.
            synced = false;
            if (imuSampled) {
                syncHeading = pose.heading();
                imuTurnSinceSync = 0.0;
                synced = true;
            }
        }
        source = Source.ODOMETRY;
        octoQuadInterrupted = false;
        state = frame;
        lastGood = frame;
        if (imuSampled) {
            updateImuReference(pose.heading(), block.velHeading_radS);
        }
    }

    private void onInvalidFrame(long now) {
        goodFrames = 0;
        if (source == Source.ODOMETRY) {
            source = Source.HOLD_LAST;
            faultStartNs = now;
        }
        if (source == Source.HOLD_LAST && !octoQuadInterrupted
                && now - faultStartNs < GLITCH_HOLD_MS * 1_000_000L) {
            state = lastGood;
            return;
        }
        useBackup(now);
    }

    // ---------- IMU ----------

    private void readImu(long now) {
        lastImuReadNs = now;
        YawPitchRollAngles angles;
        try {
            angles = imu.getRobotYawPitchRollAngles();
        } catch (RuntimeException e) {
            imuReadOk = false;
            reportImu("read failed: " + e);
            return;
        }
        double yaw = angles.getYaw(AngleUnit.RADIANS);
        // The SDK returns a zero timestamp when it could not read the IMU.
        if (angles.getAcquisitionTime() == 0 || !Double.isFinite(yaw)) {
            imuReadOk = false;
            reportImu("no data");
            return;
        }
        if (haveYaw) {
            imuTurnSinceSync += AngleUnit.normalizeRadians(yaw - lastYaw);
        }
        lastYaw = yaw;
        haveYaw = true;
        imuReadOk = true;
        imuSampled = true;
        imuProblem = null;
    }

    /** Called with a fresh IMU sample while the OctoQuad is healthy. */
    private void updateImuReference(double octoHeading, double octoTurnRate) {
        if (checkHavePrevious) {
            double octoTurn = AngleUnit.normalizeRadians(octoHeading - checkPreviousOctoHeading);
            double imuTurn = IMU_YAW_SCALAR * AngleUnit.normalizeRadians(lastYaw - checkPreviousYaw);
            checkTurned += Math.abs(octoTurn);
            checkError += Math.abs(imuTurn - octoTurn);
            if (checkTurned >= Math.toRadians(IMU_CHECK_WINDOW_DEG)) {
                ImuCheck verdict = checkError <= IMU_CHECK_MAX_ERROR_RATIO * checkTurned
                        ? ImuCheck.VERIFIED : ImuCheck.MISMATCH;
                if (verdict == ImuCheck.MISMATCH && imuCheck != ImuCheck.MISMATCH) {
                    RobotLog.addGlobalWarningMessage("IMU backup disabled: the Control Hub IMU does not"
                            + " turn with the OctoQuad. Check IMU_LOGO_DIRECTION in Panels.");
                }
                imuCheck = verdict;
                checkTurned = 0.0;
                checkError = 0.0;
            }
        }
        checkPreviousOctoHeading = octoHeading;
        checkPreviousYaw = lastYaw;
        checkHavePrevious = true;

        if (Math.abs(octoTurnRate) <= SYNC_MAX_TURN_RATE_RAD_S || !synced) {
            syncHeading = octoHeading;
            imuTurnSinceSync = 0.0;
            synced = true;
        }
    }

    private boolean imuUsable() {
        return imu != null && imuReadOk && synced && imuCheck != ImuCheck.MISMATCH
                && (imuCheck == ImuCheck.VERIFIED || USE_UNVERIFIED_IMU);
    }

    /** OctoQuad unavailable: IMU heading with frozen x/y, or a fully frozen pose. */
    private void useBackup(long now) {
        Pose frozen = state.pose();
        if (!imuUsable()) {
            source = Source.FROZEN;
            state = MotionState.ofVelocity(frozen, Velocity.zero());
            previousBackupHeading = Double.NaN;
            return;
        }
        source = Source.IMU_HEADING;
        double heading = syncHeading + IMU_YAW_SCALAR * imuTurnSinceSync;
        double turnRate = 0.0;
        if (Double.isFinite(previousBackupHeading) && now > previousBackupNs) {
            turnRate = AngleUnit.normalizeRadians(heading - previousBackupHeading)
                    / ((now - previousBackupNs) / 1e9);
        }
        previousBackupHeading = heading;
        previousBackupNs = now;
        state = MotionState.ofVelocity(new Pose(frozen.x(), frozen.y(), heading),
                new Velocity(0.0, 0.0, turnRate));
    }

    // ---------- Diagnostics ----------

    private void reportOctoQuad(String problem) {
        if (!problem.equals(octoQuadProblem)) {
            RobotLog.ww("OctoQuadImuBackup", "OctoQuad: " + problem);
        }
        octoQuadProblem = problem;
    }

    private void reportImu(String problem) {
        if (!problem.equals(imuProblem)) {
            RobotLog.ww("OctoQuadImuBackup", "IMU: " + problem);
        }
        imuProblem = problem;
    }
}
