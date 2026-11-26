package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.jetbrains.annotations.NotNull;

/** Controls the Axon Mini MK2 servo geared to the turret. */
@Configurable
public final class Turret extends SubsystemBase {
    private static final double MAX_ANGLE_DEGREES = 90.0;
    private static final double SERVO_TO_TURRET_RATIO = 1.8;
    // Nominal full sweep in Servo Mode with matching Axon programmer settings.
    private static final double SERVO_TRAVEL_DEGREES = 360.0;
    private static final double TURRET_DEGREES_PER_SERVO_RANGE =
            SERVO_TRAVEL_DEGREES / SERVO_TO_TURRET_RATIO;
    private static final PwmControl.PwmRange SERVO_PWM_RANGE =
            new PwmControl.PwmRange(500, 2500);

    // Logical servo position when the turret faces front.
    public static double CENTER_POSITION = 0.5;
    // Applied at construction to set the direction of positive angles.
    public static boolean REVERSED = false;

    private final ServoImplEx turret;
    private double commandedAngleDegrees = Double.NaN;

    public Turret(@NotNull final HardwareMap hMap) {
        this(hMap, true);
    }

    public Turret(@NotNull final HardwareMap hMap, boolean centerOnInit) {
        turret = hMap.get(ServoImplEx.class, "turret");
        turret.setPwmRange(SERVO_PWM_RANGE);
        turret.setDirection(REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        if (centerOnInit) {
            center();
        }
    }

    public void center() {
        setAngleDegrees(0.0);
    }

    /** Aims from the robot pose origin in the team Pedro frame (inches, radians, CCW positive). */
    public void aimAt(Pose robotPose, Pose target) {
        if (robotPose == null || target == null) {
            return;
        }
        double dx = target.x() - robotPose.x();
        double dy = target.y() - robotPose.y();
        if (!Double.isFinite(dx) || !Double.isFinite(dy) || (dx == 0.0 && dy == 0.0)) {
            return;
        }
        double error = Math.atan2(dy, dx) - robotPose.heading();
        setAngleDegrees(Math.toDegrees(Math.atan2(Math.sin(error), Math.cos(error))));
    }

    /** Requests an angle from robot front. Non-finite requests leave the current target in place. */
    public void setAngleDegrees(double angleDegrees) {
        double centerPosition = CENTER_POSITION;
        if (!Double.isFinite(angleDegrees) || !isValidCenter(centerPosition)) {
            return;
        }

        double limitedAngle = Range.clip(angleDegrees, -MAX_ANGLE_DEGREES, MAX_ANGLE_DEGREES);
        double offset = Range.clip(limitedAngle / TURRET_DEGREES_PER_SERVO_RANGE,
                -centerPosition, 1.0 - centerPosition);
        turret.setPosition(centerPosition + offset);
        commandedAngleDegrees = offset * TURRET_DEGREES_PER_SERVO_RANGE;
    }

    /** Returns the last commanded angle in degrees, or NaN before the first valid command. */
    public double getCommandedAngleDegrees() {
        return commandedAngleDegrees;
    }

    private static boolean isValidCenter(double position) {
        return position >= 0.0 && position <= 1.0;
    }
}
