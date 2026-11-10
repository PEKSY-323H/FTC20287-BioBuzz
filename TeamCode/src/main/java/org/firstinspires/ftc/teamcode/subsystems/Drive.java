package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.jetbrains.annotations.NotNull;

/**
 * Owns the four mecanum drive motors. Each wheel is velocity-controlled from its own motor
 * encoder, so all four turn at the same commanded speed even though their top speeds
 * differ. All four motor encoder cables must be plugged into the hub.
 */
@Configurable
public final class Drive extends SubsystemBase {
    // Single source of truth for the drive wiring: pedro/Constants reads these too, so the
    // TeleOp drive and the Pedro follower can never disagree. Positive power on every
    // wheel must roll the robot forward.
    public static final String FRONT_LEFT_NAME = "frontLeft";
    public static final String FRONT_RIGHT_NAME = "frontRight";
    public static final String BACK_LEFT_NAME = "backLeft";
    public static final String BACK_RIGHT_NAME = "backRight";
    public static final DcMotorSimple.Direction FRONT_LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction FRONT_RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction BACK_LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction BACK_RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // Wheel speed in encoder ticks per second at full input (1.0). It must stay below what
    // the slowest motor reaches at full power, or that wheel falls behind again. 2000 is
    // the drive team's figure, not measured here.
    public static double MAX_VELOCITY = 2000;

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    public Drive(@NotNull final HardwareMap hMap) {
        frontLeft = hMap.get(DcMotorEx.class, FRONT_LEFT_NAME);
        frontLeft.setDirection(FRONT_LEFT_DIRECTION);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight = hMap.get(DcMotorEx.class, FRONT_RIGHT_NAME);
        frontRight.setDirection(FRONT_RIGHT_DIRECTION);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeft = hMap.get(DcMotorEx.class, BACK_LEFT_NAME);
        backLeft.setDirection(BACK_LEFT_DIRECTION);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRight = hMap.get(DcMotorEx.class, BACK_RIGHT_NAME);
        backRight.setDirection(BACK_RIGHT_DIRECTION);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stop();
    }

    /** Sets normalized robot-centric forward, right-strafe, and turn inputs. */
    public void drive(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;
        double scale = Math.max(1.0, Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));

        frontLeft.setVelocity(fl / scale * MAX_VELOCITY);
        frontRight.setVelocity(fr / scale * MAX_VELOCITY);
        backLeft.setVelocity(bl / scale * MAX_VELOCITY);
        backRight.setVelocity(br / scale * MAX_VELOCITY);
    }

    public void stop() {
        frontLeft.setVelocity(0.0);
        frontRight.setVelocity(0.0);
        backLeft.setVelocity(0.0);
        backRight.setVelocity(0.0);
    }

}
