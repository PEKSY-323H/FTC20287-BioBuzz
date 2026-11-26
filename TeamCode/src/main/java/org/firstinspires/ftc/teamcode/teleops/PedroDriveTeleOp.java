package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.autos.Points;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.GamepadConfig;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.AutoHandoff;

import java.util.Locale;

/**
 * Shared Pedro driving for both TeleOps: one Pedro follower does manual drive, the B/X
 * shooting-position paths and the OctoQuad odometry. Starts from the pose where AUTO ended
 * (AutoHandoff) when there is one; gamepad 1 Y resets the odometry to the alliance's
 * reset pose. Subclasses only decide how stick input becomes drive power.
 */
@Configurable
public abstract class PedroDriveTeleOp extends BaseDriveTeleOp {
    // Set these in Panels before INIT. Heading is CCW from field +x, in degrees.
    public static boolean USE_AUTO_HANDOFF = true;
    public static boolean START_ON_BLUE = false;
    public static double START_X_IN = 70.75;
    public static double START_Y_IN = 70.75;
    public static double START_HEADING_DEG = 0.0;

    // Gamepad 1 Y resets the odometry to this pose for the current alliance, e.g. with the
    // robot pushed against a known spot. Field frame, inches and degrees.
    // TODO: placeholders, fill in the measured reset poses.
    public static double RED_RESET_X_IN = 0.0;
    public static double RED_RESET_Y_IN = 0.0;
    public static double RED_RESET_HEADING_DEG = 0.0;
    public static double BLUE_RESET_X_IN = 0.0;
    public static double BLUE_RESET_Y_IN = 0.0;
    public static double BLUE_RESET_HEADING_DEG = 0.0;

    protected Follower follower;
    private GamepadConfig controls;
    private Alliance alliance;
    private boolean usingHandoff;
    private boolean shoot1WasHeld;
    private boolean shoot2WasHeld;
    private boolean resetWasHeld;
    private AutoHandoff handoff;

    /**
     * Sends the driver's input to the follower. forward is robot/driver-frame +x, lateral is
     * left-positive and turn is CCW-positive (Pedro's conventions), already deadbanded and scaled.
     */
    protected abstract void driveManually(double forward, double lateral, double turn);

    /** One line for the INIT screen on what stick-forward means in this TeleOp. */
    protected abstract String stickFrameHint();

    /** Returns true when manual drive should take over immediately after a mode change. */
    protected boolean updateDriveMode(GamepadConfig controls) {
        return false;
    }

    /** AUTO uses the alliance driver frame; standalone driving uses heading zero as forward. */
    protected final double headingFromDriverForward() {
        return follower.pose().heading() - (usingHandoff ? alliance.driverForwardHeading() : 0.0);
    }

    protected final boolean isUsingAutoHandoff() {
        return usingHandoff;
    }

    @Override
    protected final void initializeDrive(GamepadConfig controls) {
        this.controls = controls;
        handoff = AutoHandoff.latest();
        usingHandoff = USE_AUTO_HANDOFF && handoff != null;
        alliance = usingHandoff ? handoff.alliance : (START_ON_BLUE ? Alliance.BLUE : Alliance.RED);
        Pose startPose = usingHandoff ? handoff.pose
                : new Pose(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG));

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.holdEnd.set(false);
    }

    @Override
    protected final void updateDriveInitialization() {
        if (!usingHandoff) {
            if (controls.isBlueAllianceSelected() && !controls.isRedAllianceSelected()) {
                START_ON_BLUE = true;
            } else if (controls.isRedAllianceSelected() && !controls.isBlueAllianceSelected()) {
                START_ON_BLUE = false;
            }
            alliance = START_ON_BLUE ? Alliance.BLUE : Alliance.RED;
            telemetry.addLine("INIT alliance: X = BLUE, B = RED (or set START_ON_BLUE in Panels).");
        }
        // Selecting an alliance during INIT must not launch a shooting path at START.
        shoot1WasHeld = controls.isDriveToShoot1Held();
        shoot2WasHeld = controls.isDriveToShoot2Held();
        follower.localizer.update();
        telemetry.addData("Start pose source", usingHandoff ? "AUTO handoff" : "configured");
        telemetry.addData("Alliance", alliance);
        if (usingHandoff) {
            telemetry.addData("AUTO ended at", String.format(Locale.US, "(%.1f, %.1f, %.0f deg), %.0f s ago",
                    handoff.pose.x(), handoff.pose.y(), Math.toDegrees(handoff.pose.heading()),
                    AutoHandoff.secondsSinceRecorded()));
        }
        telemetry.addLine("Y: reset odometry to the " + alliance + " reset pose.");
        telemetry.addLine("B: shoot1. X: shoot2. Move a drive stick to cancel.");
        telemetry.addData("B shooting target", shootingTarget(true));
        telemetry.addData("X shooting target", shootingTarget(false));
        telemetry.addLine(stickFrameHint());
    }

    @Override
    protected final void beforeDriveRun() {
        boolean driveModeChanged = updateDriveMode(controls);
        double forward = applyDeadband(controls.getForward()) * GamepadConfig.DRIVE_LIMIT;
        // Pedro uses left-positive strafe and CCW-positive turn.
        double lateral = -applyDeadband(controls.getStrafe()) * GamepadConfig.DRIVE_LIMIT;
        double turn = -applyDeadband(controls.getTurn()) * GamepadConfig.TURN_LIMIT;
        boolean driverMoving = forward != 0 || lateral != 0 || turn != 0;
        boolean resetHeld = controls.isResetPoseHeld();
        if (resetHeld && !resetWasHeld) {
            resetPose();
        }
        resetWasHeld = resetHeld;

        boolean shoot1Held = controls.isDriveToShoot1Held();
        boolean shoot2Held = controls.isDriveToShoot2Held();

        if (!driverMoving && !driveModeChanged) {
            // Read this alliance's shared points now so Panels tuning is reflected in each new path.
            if (shoot1Held && !shoot1WasHeld) {
                followTo(shootingTarget(true));
            } else if (shoot2Held && !shoot2WasHeld) {
                followTo(shootingTarget(false));
            }
        }
        shoot1WasHeld = shoot1Held;
        shoot2WasHeld = shoot2Held;

        if (driveModeChanged || driverMoving || follower.mode() != Follower.Mode.FOLLOW) {
            driveManually(forward, lateral, turn);
        }
        follower.update();
    }

    @Override
    protected final void aimTurret(Turret turret) {
        Pose robotPose = follower.pose();
        if (robotPose != null) {
            turret.aimAt(robotPose, Points.cellTarget(alliance, robotPose.y()));
        }
    }

    private Pose shootingTarget(boolean first) {
        if (alliance == Alliance.BLUE) {
            return (first ? Points.Blue.SHOOT_1 : Points.Blue.SHOOT_2).toPose();
        }
        return (first ? Points.Red.SHOOT_1 : Points.Red.SHOOT_2).toPose();
    }

    private void followTo(Pose target) {
        Pose start = follower.pose();
        follower.follow(Paths.line(start, target)
                .heading(Interpolator.linear(start.heading(), target.heading())));
    }

    /** The configured reset pose for the current alliance. */
    private Pose allianceResetPose() {
        return alliance == Alliance.BLUE
                ? new Pose(BLUE_RESET_X_IN, BLUE_RESET_Y_IN, Math.toRadians(BLUE_RESET_HEADING_DEG))
                : new Pose(RED_RESET_X_IN, RED_RESET_Y_IN, Math.toRadians(RED_RESET_HEADING_DEG));
    }

    /**
     * Overwrites the odometry (OctoQuad) pose with the alliance's reset pose, drops any B/X path
     * that was running (it was planned from the old pose) and rumbles gamepad 1 to confirm.
     */
    private void resetPose() {
        follower.setPose(allianceResetPose());
        if (follower.mode() == Follower.Mode.FOLLOW) {
            follower.manual(0, 0, 0);
        }
        controls.rumble(200);
    }

    private double applyDeadband(double input) {
        double deadband = GamepadConfig.DRIVE_DEADBAND;
        if (Math.abs(input) <= deadband) {
            return 0;
        }
        return Math.copySign((Math.abs(input) - deadband) / (1 - deadband), input);
    }

    @Override
    protected final void stopDrive() {
        follower.stop();
        follower.drivetrain.stop();
    }

    @Override
    protected final void addDriveTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("Pedro", follower.mode());
        telemetry.addData("Drive frame", stickFrameHint());
        telemetry.addData("Alliance", alliance);
        Pose reset = allianceResetPose();
        telemetry.addData("Y reset pose", String.format(Locale.US, "(%.1f, %.1f, %.0f deg)",
                reset.x(), reset.y(), Math.toDegrees(reset.heading())));
        telemetry.addData("Field x (in)", pose.x());
        telemetry.addData("Field y (in)", pose.y());
        telemetry.addData("Field heading (deg)", Math.toDegrees(pose.heading()));
    }
}
