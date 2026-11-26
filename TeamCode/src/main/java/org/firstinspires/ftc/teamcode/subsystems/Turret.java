package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.jetbrains.annotations.NotNull;

/** Controls the Axon Mini MK2 servo geared to the turret. */
@Configurable
public final class Turret extends SubsystemBase {
    private static final double CENTER_ANGLE_DEGREES = 180.0;
    private static final double MAX_OFFSET_DEGREES = 90.0;
    private static final double SERVO_TO_TURRET_RATIO = 1.8;
    // Nominal full sweep in Servo Mode with matching Axon programmer settings.
    private static final double SERVO_TRAVEL_DEGREES = 360.0;
    private static final double TURRET_DEGREES_PER_SERVO_RANGE =
            SERVO_TRAVEL_DEGREES / SERVO_TO_TURRET_RATIO;
    private static final PwmControl.PwmRange SERVO_PWM_RANGE =
            new PwmControl.PwmRange(500, 2500);

    // Logical servo position when the turret faces the rear of the robot.
    public static double CENTER_POSITION = 0.5;
    // Applied at construction to set the direction of positive angles.
    public static boolean REVERSED = false;
    // Targets within this many degrees of dead-front (unreachable either way) hold the
    // current limit side. 0 disables; capped at 90 so reachable angles are never affected.
    public static double FRONT_HYSTERESIS_DEG = 30.0;

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
        setAngleDegrees(CENTER_ANGLE_DEGREES);
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
        double robotRelativeBearing = Math.atan2(dy, dx) - robotPose.heading();
        setAngleDegrees(Math.toDegrees(robotRelativeBearing));
    }

    /** Requests a robot-relative angle, measured CCW from 0° front and limited to 90°–270°. */
    public void setAngleDegrees(double angleDegrees) {
        double centerPosition = CENTER_POSITION;
        if (!Double.isFinite(angleDegrees) || !isValidCenter(centerPosition)) {
            return;
        }

        double rearOffsetDegrees = AngleUnit.normalizeDegrees(angleDegrees % 360.0 - CENTER_ANGLE_DEGREES);
        double limitedOffsetDegrees = Range.clip(rearOffsetDegrees, -MAX_OFFSET_DEGREES, MAX_OFFSET_DEGREES);
        // Near dead-front the target is out of reach on both sides and the sign of the error
        // flips with heading noise; keep the limit the turret is already on instead of
        // swinging 180 degrees to the other one.
        double band = Range.clip(FRONT_HYSTERESIS_DEG, 0.0, 180.0 - MAX_OFFSET_DEGREES);
        double currentOffsetDegrees = commandedAngleDegrees - CENTER_ANGLE_DEGREES;
        if (Math.abs(rearOffsetDegrees) > 180.0 - band && currentOffsetDegrees != 0.0
                && Double.isFinite(currentOffsetDegrees)) {
            limitedOffsetDegrees = Math.copySign(MAX_OFFSET_DEGREES, currentOffsetDegrees);
        }
        double offset = Range.clip(limitedOffsetDegrees / TURRET_DEGREES_PER_SERVO_RANGE,
                -centerPosition, 1.0 - centerPosition);
        turret.setPosition(centerPosition + offset);
        commandedAngleDegrees = CENTER_ANGLE_DEGREES + offset * TURRET_DEGREES_PER_SERVO_RANGE;
    }

    /** Returns the last commanded robot-relative angle, or NaN before the first valid command. */
    public double getCommandedAngleDegrees() {
        return commandedAngleDegrees;
    }

    private static boolean isValidCenter(double position) {
        return position >= 0.0 && position <= 1.0;
    }

    public void lockTurret(){
        setAngleDegrees(CENTER_ANGLE_DEGREES);
    }
}
