package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.jetbrains.annotations.NotNull;

/** Owns the intake motor and continuous-rotation splitter servo. */
@Configurable
public final class Intake extends SubsystemBase {
    // Shared commanded power, limited to [-1, 1] by setPower(); tune in Panels.
    public static double INTAKE_POWER = 1.0;
    // Shared shooting/feed power for every auto and TeleOp; collection uses INTAKE_POWER.
    public static double SHOOT_INTAKE_POWER = 1;
    // Holds collected POLLEN in the intake while driving to a shooting position.
    public static double INTAKE_SLOW_POWER = 0.25;
    public static double OUTTAKE_POWER = -1.0;
    // CR servo PWM commands: 0.5 is neutral; either side controls direction/speed.
    public static double SPLITTER_DEPLOY_POSITION = 1;
    public static double SPLITTER_RETRACT_POSITION = 0;
    public static double SPLITTER_STOP_POSITION = 0.5;
    // Measure each direction's travel time on the robot before use.
    public static double SPLITTER_DEPLOY_SECONDS = 0.75;
    public static double SPLITTER_RETRACT_SECONDS = 0.76;
    private final Servo splitter;
    private long splitterDeadline;
    private final DcMotorEx intake;
    private double commandedPower;

    public Intake(@NotNull final HardwareMap hMap) {
        intake = hMap.get(DcMotorEx.class, "intake");
        splitter = hMap.get(Servo.class, "splitter");
        splitter.setDirection(Servo.Direction.FORWARD);
        stopSplitter();
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stop();
    }

    public void runIntake() {
        setPower(INTAKE_POWER);
    }

    /** Shared feed power, independent of collection and loaded transport power. */
    public void runIntakeForShooting() {
        setPower(SHOOT_INTAKE_POWER);

    }

    public void runIntakeSlow() {
        setPower(INTAKE_SLOW_POWER);
    }

    public void runOuttake() {
        setPower(OUTTAKE_POWER);
    }

    public void setPower(double power) {
        double limitedPower = Math.max(-1.0, Math.min(1.0, power));
        intake.setPower(limitedPower);
        commandedPower = limitedPower;
    }

    public void stop() {
        setPower(0.0);
    }

    public boolean deploySplitter() {
        return setSplitterPosition(SPLITTER_DEPLOY_POSITION, SPLITTER_DEPLOY_SECONDS);
    }

    public boolean retractSplitter() {
        return setSplitterPosition(SPLITTER_RETRACT_POSITION, SPLITTER_RETRACT_SECONDS);
    }

    /** Returns whether a timed movement was started; active movements cannot be replaced. */
    private boolean setSplitterPosition(double position, double seconds) {
        if (isSplitterMoving()) {
            return false;
        }
        if (!Double.isFinite(position) || !Double.isFinite(seconds) || seconds <= 0) {
            stopSplitter();
            return false;
        }
        splitter.setPosition(Math.max(0.0, Math.min(1.0, position)));
        splitterDeadline = System.nanoTime() + (long) (seconds * 1e9);
        return true;
    }

    /** Timer state only, not measured mechanism motion. */
    public boolean isSplitterMoving() {
        return splitterDeadline != 0;
    }

    public void stopSplitter() {
        double neutral = Double.isFinite(SPLITTER_STOP_POSITION) ? SPLITTER_STOP_POSITION : 0.5;
        splitter.setPosition(Math.max(0.0, Math.min(1.0, neutral)));
        splitterDeadline = 0;
    }

    @Override
    public void periodic() {
        if (splitterDeadline != 0 && System.nanoTime() - splitterDeadline >= 0) {
            stopSplitter();
        }
    }

    /** Last successfully written command, not a measurement of mechanism motion. */
    public double getCommandedPower() {
        return commandedPower;
    }

    public double getPower() {
        return intake.getPower();
    }
}
