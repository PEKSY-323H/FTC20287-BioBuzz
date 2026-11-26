package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PathFollower;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.AutoHandoff;

import java.util.LinkedHashSet;
import java.util.Locale;
import java.util.Set;

/**
 * Shared lifecycle for Pedro + OctoQuad autonomous routines on SolversLib, built around
 * the BIOBUZZ (2026-2027) rules in Competition Manual TU08.
 *
 * <p>Subclasses may write BLUE poses with {@link #pose} or direct RED field poses with
 * {@link #redPose}. Implement {@link #startPose()} and {@link #buildRoutine()}; the routine is
 * built during INIT and scheduled at START.
 *
 * <p>What the rules ask of an AUTO, and what this class does about it:
 * <ul>
 * <li>G304 setup: fully on our own side, touching the perimeter wall, outside the LOADING
 *     ZONE, exactly 4 POLLEN preloaded, motionless after INIT. Nothing moves the drivetrain
 *     before START; the pose is re-applied at START in case the robot was nudged.</li>
 * <li>Scoring: LEAVE 3 (not touching the perimeter wall at the end of AUTO), PARK 5 (partly
 *     in our LOADING ZONE), HIVE TIP 20. LEAVE and PARK also count toward the SWARM RP, so
 *     reserve time for the last move with {@link #untilSecondsLeft}.</li>
 * <li>G402: disrupting the opponent's side during AUTO is a MAJOR FOUL. {@link #pose}
 *     warns on the Driver Station about poses near or past the centre line.</li>
 * <li>G403: no powered movement between AUTO and TELEOP. Everything is stopped
 *     END_GUARD_SECONDS before the 30 s mark and again when the OpMode ends.</li>
 * <li>G407: a robot may control at most 4 SCORING ELEMENTS. Nothing here counts them, so
 *     a routine must not intake past 4.</li>
 * </ul>
 */
@Configurable
public abstract class BaseAuto extends CommandOpMode {
    /** AUTO length in seconds (section 10.4); an 8 s transition follows. */
    public static final double AUTO_SECONDS = 30.0;
    // Fraction of Pedro's tuned max path speed. Keep low for first runs; raise after tuning.
    public static double PATH_SPEED_FRACTION = 0.7;
    // A path or turn that has not finished by then is abandoned and the robot holds its pose.
    public static long DEFAULT_PATH_TIMEOUT_MS = 5000;
    // Stop everything this long before the end of AUTO so nothing is still powered at 30 s.
    public static double END_GUARD_SECONDS = 0.2;
    // Warn about poses whose robot centre is closer than this to the centre line (inches);
    // half the robot's size keeps the whole robot on our side.
    public static double OWN_SIDE_MARGIN_IN = 9.0;
    // settleAt(): the robot counts as arrived once it is this close to the target pose.
    public static double SETTLE_POSITION_TOLERANCE_IN = 1.0;
    public static double SETTLE_HEADING_TOLERANCE_DEG = 3.0;
    // settleAt() gives up waiting after this long and lets the routine carry on.
    public static long SETTLE_TIMEOUT_MS = 1500;

    protected final Alliance alliance;
    protected PathFollower follower;
    protected Shooter shooter;
    protected Intake intake;
    protected Gate gate;
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final Set<String> setupWarnings = new LinkedHashSet<>();
    private Pose startFieldPose;
    private Command routine;
    private boolean endGuardTripped;

    protected BaseAuto(Alliance alliance) {
        this.alliance = alliance;
    }

    /** Starting pose in the team field frame. */
    protected abstract Pose startPose();

    /** Builds the whole autonomous as one command. Called once during INIT. */
    protected abstract Command buildRoutine();

    @Override
    public final void initialize() {
        // An OpMode that crashed in INIT never reaches CommandOpMode's own reset and would
        // leave its subsystems registered; start from a clean scheduler.
        reset();
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        endGuardTripped = false;
        setupWarnings.clear();

        telemetry.addLine("Calibrating OctoQuad IMU: keep the robot still...");
        telemetry.update();
        startFieldPose = startPose();
        // Blocks until the OctoQuad reports RUNNING.
        follower = new PathFollower(hardwareMap, startFieldPose);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        // The gate servo closes here, inside the starting configuration; nothing else moves.
        gate = new Gate(hardwareMap);
        routine = buildRoutine();
    }

    @Override
    public final void initialize_loop() {
        // No commands are scheduled yet: this only refreshes the localizer (follower is
        // IDLE, so drive output stays zero) for placement telemetry.
        super.run();
        telemetry.addData("Alliance", alliance);
        telemetry.addLine("G304: own side, touching the wall, outside the LOADING ZONE,");
        telemetry.addLine("4 POLLEN preloaded. The pose below resets to the start pose at START.");
        addPoseTelemetry();
        addWarningTelemetry();
        telemetry.update();
    }

    @Override
    public final void preRun() {
        // Discard whatever the odometry picked up while the robot was being placed.
        follower.setPose(startFieldPose);
        autoTimer.reset();
        // Routines explicitly select collection, loaded transport, or shooting power.
        intake.stop();
        schedule(routine);
    }

    @Override
    public final void run() {
        if (!endGuardTripped && secondsLeft() <= END_GUARD_SECONDS) {
            endGuardTripped = true;
            CommandScheduler.getInstance().cancelAll();
            stopAll();
        }
        super.run();

        telemetry.addData("Alliance", alliance);
        telemetry.addData("AUTO time left (s)", secondsLeft());
        telemetry.addData("Routine running", routine.isScheduled());
        telemetry.addData("Splitter timer active", intake.isSplitterMoving());
        addPoseTelemetry();
        telemetry.addData("Follower mode", follower.getMode());
        telemetry.addData("Path completion", follower.getCompletion());
        telemetry.addData("Shooter Velocity Command", shooter.getCommandedVelocity());
        telemetry.addData("Shooter Encoder Mismatch", shooter.isEncoderMismatch());
        telemetry.addData("Intake Power Command", intake.getCommandedPower());
        telemetry.addData("Gate Position Command", gate.getPosition());
        addWarningTelemetry();
        telemetry.update();
    }

    @Override
    public final void end() {
        // Lets the Field-Centric TeleOp keep "forward = away from the drivers".
        AutoHandoff.record(follower.getPose(), alliance);
        CommandScheduler.getInstance().cancelAll();
        stopAll();
    }

    /** Attempts every stop even if one of them throws, then rethrows the first failure. */
    private void stopAll() {
        Runnable[] stops = {follower::stop, shooter::stop, intake::stop, intake::stopSplitter, gate::close};
        RuntimeException failure = null;
        for (Runnable stop : stops) {
            try {
                stop.run();
            } catch (RuntimeException e) {
                if (failure == null) {
                    failure = e;
                }
            }
        }
        if (failure != null) {
            throw failure;
        }
    }

    private void addPoseTelemetry() {
        Pose p = follower.getPose();
        if (p == null) {
            telemetry.addData("Pose", "UNAVAILABLE");
            return;
        }
        telemetry.addData("x (in)", p.x());
        telemetry.addData("y (in)", p.y());
        telemetry.addData("heading (deg)", Math.toDegrees(p.heading()));
    }

    private void addWarningTelemetry() {
        for (String warning : setupWarnings) {
            telemetry.addData("WARNING", warning);
        }
    }

    // ---------- Helpers for subclasses ----------

    /**
     * BLUE-side pose in inches and degrees, converted to the selected alliance. BLUE owns
     * x &gt; 70.75; a pose closer than OWN_SIDE_MARGIN_IN to the centre line is still returned
     * but reported as a G402 warning. The HIVE structure also stands on the centre line.
     */
    protected final Pose pose(double x, double y, double headingDegrees) {
        if (x < Alliance.FIELD_SIZE / 2 + OWN_SIDE_MARGIN_IN) {
            setupWarnings.add(String.format(Locale.US,
                    "G402: BLUE pose (%.1f, %.1f) is within %.1f in of the opponent side",
                    x, y, OWN_SIDE_MARGIN_IN));
        }
        return alliance.fromBlue(new Pose(x, y, Math.toRadians(headingDegrees)));
    }

    /** RED-side pose in the team field frame, with an own-side warning. */
    protected final Pose redPose(double x, double y, double headingDegrees) {
        if (x > Alliance.FIELD_SIZE / 2 - OWN_SIDE_MARGIN_IN) {
            setupWarnings.add(String.format(Locale.US,
                    "G402: RED pose (%.1f, %.1f) is within %.1f in of the opponent side",
                    x, y, OWN_SIDE_MARGIN_IN));
        }
        return new Pose(x, y, Math.toRadians(headingDegrees));
    }

    /**
     * A RED-side pose from a {@link Points} entry, with the same own-side warning. Reads the
     * point's current (possibly Panels-tuned) values, so call it while building the routine.
     */
    protected final Pose redPose(Points.Point p) {
        return redPose(p.x, p.y, p.headingDeg);
    }

    /**
     * A BLUE-side pose from a {@link Points} entry, used as-is in the field frame (no alliance
     * conversion), with the matching own-side warning.
     */
    protected final Pose bluePose(Points.PoseSource source) {
        Pose p = source.toPose();
        if (p.x() < Alliance.FIELD_SIZE / 2 + OWN_SIDE_MARGIN_IN) {
            setupWarnings.add(String.format(Locale.US,
                    "G402: BLUE pose (%.1f, %.1f) is within %.1f in of the opponent side",
                    p.x(), p.y(), OWN_SIDE_MARGIN_IN));
        }
        return p;
    }

    /** Straight line, heading interpolated from start to end. */
    protected final Path line(Pose start, Pose end) {
        return Paths.line(start, end).linear(start, end);
    }

    /** Bezier curve through control points, heading interpolated first → last. */
    protected final Path curve(Pose... controlPoints) {
        Pose start = controlPoints[0];
        Pose end = controlPoints[controlPoints.length - 1];
        return Paths.curve(controlPoints).linear(start, end);
    }

    /** A line with an explicit heading profile from the route editor. */
    protected final Path line(Pose start, Pose end, Interpolator heading) {
        return Paths.line(start, end).heading(heading);
    }

    /** A quadratic Bezier with an explicit heading profile from the route editor. */
    protected final Path curve(Pose start, Pose control, Pose end, Interpolator heading) {
        return Paths.curve(start, control, end).heading(heading);
    }

    /**
     * Follows a path at PATH_SPEED_FRACTION of max speed, then holds the end pose.
     * Finishes once the robot settles at the end, or after the timeout.
     */
    protected final Command follow(Path path) {
        return follow(path, PATH_SPEED_FRACTION, DEFAULT_PATH_TIMEOUT_MS);
    }

    /** speedFraction is a fraction (0, 1] of max path speed, not a motor power cap. */
    protected final Command follow(Path path, double speedFraction, long timeoutMs) {
        return new FollowPathCommand(follower.getFollower(), path, speedFraction)
                .addRequirements(follower)
                .withTimeout(timeoutMs);
    }

    /** Turns in place to a BLUE-side field heading in degrees (converted for RED). */
    protected final Command turnTo(double blueHeadingDegrees) {
        double heading = alliance.headingFromBlue(Math.toRadians(blueHeadingDegrees));
        return new TurnToCommand(follower.getFollower(), heading, AngleUnit.RADIANS)
                .setHeadingTolerance(2, AngleUnit.DEGREES)
                .addRequirements(follower)
                .withTimeout(DEFAULT_PATH_TIMEOUT_MS);
    }

    /**
     * Waits until the robot is within SETTLE_POSITION_TOLERANCE_IN and SETTLE_HEADING_TOLERANCE_DEG
     * of {@code target}, or SETTLE_TIMEOUT_MS. A follow() can finish while the robot is still
     * turning (Pedro ends a path 100 ms after reaching its end point); the follower keeps holding
     * the end pose meanwhile, so this lets it get there before, for example, the gate opens.
     */
    protected final Command settleAt(Pose target) {
        return new WaitUntilCommand(() -> isAt(target)).withTimeout(SETTLE_TIMEOUT_MS);
    }

    private boolean isAt(Pose target) {
        Pose p = follower.getPose();
        if (p == null) {
            return false;
        }
        double distance = Math.hypot(p.x() - target.x(), p.y() - target.y());
        double headingDiff = p.heading() - target.heading();
        double headingError = Math.abs(Math.atan2(Math.sin(headingDiff), Math.cos(headingDiff)));
        return distance <= SETTLE_POSITION_TOLERANCE_IN
                && headingError <= Math.toRadians(SETTLE_HEADING_TOLERANCE_DEG);
    }

    /** Seconds since START. */
    protected final double matchSeconds() {
        return autoTimer.seconds();
    }

    /** Seconds of AUTO remaining; negative once the 30 s have passed. */
    protected final double secondsLeft() {
        return AUTO_SECONDS - matchSeconds();
    }

    /**
     * Runs a command but interrupts it once only {@code secondsLeft} of AUTO remain, so
     * the commands after it (LEAVE the wall, PARK in the LOADING ZONE) always get their time:
     * <pre>
     * new SequentialCommandGroup(untilSecondsLeft(cycles, 4.0), follow(toLoadingZone))
     * </pre>
     */
    protected final Command untilSecondsLeft(Command command, double secondsLeft) {
        return command.interruptOn(() -> secondsLeft() <= secondsLeft);
    }

    protected final Command spinUp() {
        return new InstantCommand(shooter::shoot, shooter);
    }

    /** Starts timed retraction without waiting, so it can run during the final path. */
    protected final Command retractSplitter() {
        return new InstantCommand(() -> {
            // A park cutoff may interrupt deployment; allow retraction to replace it.
            intake.stopSplitter();
            intake.retractSplitter();
        }, intake);
    }

    protected final Command intakeOn() {
        return new InstantCommand(intake::runIntake, intake);
    }

    /** Holds loaded pollen while travelling or waiting, without feeding at full power. */
    protected final Command intakeSlow() {
        return new InstantCommand(intake::runIntakeSlow, intake);
    }

    protected final Command intakeOff() {
        return new InstantCommand(intake::stop, intake);
    }

    /**
     * Matches TeleOp RT: spins up with the gate closed and intake stopped, then feeds only
     * while canOpenGate() is true. The shooting window lasts feedSeconds after first readiness;
     * speed dips close the gate and stop the intake without extending that window. If readiness
     * never arrives, the routine's parking reserve/end guard interrupts the wait. Completion or
     * interruption stops the intake and flywheel and closes the gate.
     */
    protected final Command shoot(double feedSeconds) {
        return new SequentialCommandGroup(
                spinUp(),
                new InstantCommand(() -> {
                    shooter.stopPreshooter();
                    gate.close();
                    intake.stop();
                }, shooter, gate, intake),
                new WaitUntilCommand(shooter::canOpenGate),
                new RunCommand(() -> {
                    if (shooter.canOpenGate()) {
                        gate.open();
                        intake.runIntakeForShooting();
                        shooter.runPreshooter();
                    } else {
                        shooter.stopPreshooter();
                        gate.close();
                        intake.stop();
                    }
                }, shooter, gate, intake).withTimeout((long) (Math.max(0.0, feedSeconds) * 1000))) {
            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                // Collection and loaded transport commands choose the next intake power.
                intake.stop();
                gate.close();
                shooter.stop();
            }
        };
    }
}
