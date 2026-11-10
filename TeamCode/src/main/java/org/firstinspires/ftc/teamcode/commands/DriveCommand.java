package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Localizer;
import org.jetbrains.annotations.NotNull;

import java.util.function.DoubleSupplier;

/** Continuously sends live driver inputs to the drivetrain. */
public final class DriveCommand extends CommandBase {
    private final Drive drive;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;
    private final DoubleSupplier heading;
    private final double deadband;
    private final double driveLimit;
    private final double turnLimit;
    private DoubleSupplier turnOverride = () -> Double.NaN;

    /** Robot-centric drive; no heading sensor is required. */
    public DriveCommand(@NotNull final Drive drive, DoubleSupplier forward,
                        DoubleSupplier strafe, DoubleSupplier turn,
                        double deadband, double driveLimit, double turnLimit) {
        this(drive, forward, strafe, turn, () -> 0.0, deadband, driveLimit, turnLimit);
    }

    /**
     * Field-centric drive. While the localizer has no heading (before the reference is
     * set, or after the OctoQuad drops out) the sticks act robot-centric instead of
     * stopping the robot.
     */
    public DriveCommand(@NotNull final Drive drive, DoubleSupplier forward,
                        DoubleSupplier strafe, DoubleSupplier turn,
                        Localizer localizer, double deadband, double driveLimit, double turnLimit) {
        this(drive, forward, strafe, turn, localizer::getHeadingRadians, deadband,
                driveLimit, turnLimit);
        addRequirements(localizer);
    }

    private DriveCommand(@NotNull final Drive drive, DoubleSupplier forward,
                         DoubleSupplier strafe, DoubleSupplier turn,
                         DoubleSupplier heading, double deadband, double driveLimit, double turnLimit) {
        this.drive = drive;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        this.heading = heading;
        this.deadband = deadband;
        this.driveLimit = driveLimit;
        this.turnLimit = turnLimit;
        addRequirements(drive);
    }

    /**
     * Lets an assist (auto-aim) replace the driver's turn. Each loop a finite value is
     * used as the turn power as-is (positive = clockwise, no deadband or turn limit);
     * NaN hands the turn back to the driver's stick.
     */
    public DriveCommand withTurnOverride(@NotNull final DoubleSupplier turnOverride) {
        this.turnOverride = turnOverride;
        return this;
    }

    @Override
    public void execute() {
        double headingRadians = heading.getAsDouble();
        if (!Double.isFinite(headingRadians)) {
            // Zero heading makes the rotation below the identity: robot-centric fallback.
            headingRadians = 0.0;
        }

        double forwardValue = applyDeadband(forward.getAsDouble()) * driveLimit;
        double strafeValue = applyDeadband(strafe.getAsDouble()) * driveLimit;
        double overrideValue = turnOverride.getAsDouble();
        double turnValue = Double.isFinite(overrideValue)
                ? Math.max(-1.0, Math.min(1.0, overrideValue))
                : applyDeadband(turn.getAsDouble()) * turnLimit;
        double cosine = Math.cos(headingRadians);
        double sine = Math.sin(headingRadians);
        double robotForward = forwardValue * cosine - strafeValue * sine;
        double robotStrafe = forwardValue * sine + strafeValue * cosine;
        drive.drive(robotForward, robotStrafe, turnValue);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    private double applyDeadband(double input) {
        if (Math.abs(input) <= deadband) {
            return 0.0;
        }
        return Math.copySign((Math.abs(input) - deadband) / (1.0 - deadband), input);
    }
}
