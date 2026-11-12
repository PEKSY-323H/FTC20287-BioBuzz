package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.jetbrains.annotations.NotNull;

/** Owns the intake motor; the TeleOp decides when to collect. */
@Configurable
public final class Intake extends SubsystemBase {
    // Shared commanded power, limited to [-1, 1] by setPower(); tune in Panels.
    public static double INTAKE_POWER = 1.0;
    public static double OUTTAKE_POWER = -1.0;
    // Measure the travel time and direction on the robot before using the splitter.
    public static double SPLITTER_DEPLOY_POWER = 1.0;
    public static double SPLITTER_RETRACT_POWER = -1.0;
    public static double SPLITTER_SECONDS = 0.0;
    private final CRServo splitter;
    private long splitterDeadline;
    private final DcMotorEx intake;
    private double commandedPower;

    public Intake(@NotNull final HardwareMap hMap) {
        intake = hMap.get(DcMotorEx.class, "intake");
        splitter = hMap.get(CRServo.class, "splitter");
        stopSplitter();
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stop();
    }

    public void runIntake() {
        setPower(INTAKE_POWER);
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

    public void deploySplitter() {
        moveSplitter(SPLITTER_DEPLOY_POWER, SPLITTER_SECONDS);
    }

    public void retractSplitter() {
        moveSplitter(SPLITTER_RETRACT_POWER, SPLITTER_SECONDS);
    }

    private void moveSplitter(double power, double seconds) {
        if (!Double.isFinite(power) || !Double.isFinite(seconds) || seconds <= 0) {
            stopSplitter();
            return;
        }
        splitterDeadline = System.nanoTime() + (long) (seconds * 1e9);
        splitter.setPower(Math.max(-1.0, Math.min(1.0, power)));
    }

    public void stopSplitter() {
        splitter.setPower(0);
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

    public double getPow() { return intake.getPower(); }
}
