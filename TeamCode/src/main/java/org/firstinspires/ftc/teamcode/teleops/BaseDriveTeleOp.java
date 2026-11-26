package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.GamepadConfig;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/** Shared hardware, controls, and lifecycle for drive TeleOps. */
public abstract class BaseDriveTeleOp extends CommandOpMode {

    private Shooter shooter;
    private Intake intake;
    private Gate gate;
    private Turret turret;
    private GamepadConfig controls;
    private boolean splitterButtonWasHeld;
    private boolean splitterDeployed;

    @Override
    public final void initialize() {
        // An OpMode that crashed in INIT never reaches CommandOpMode's own reset and would
        // leave its subsystems registered; start from a clean scheduler.
        reset();
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        gate = new Gate(hardwareMap);
        turret = new Turret(hardwareMap, false);
        controls = new GamepadConfig(gamepad1);
        // Ignore D-pad up held at START; require release, then a fresh press.
        splitterButtonWasHeld = true;
        splitterDeployed = false;
        initializeDrive(controls);
    }

    protected abstract void initializeDrive(GamepadConfig controls);

    @Override
    public final void initialize_loop() {
        updateDriveInitialization();
        telemetry.update();
        sleep(50);
    }

    protected void updateDriveInitialization() {
        telemetry.addLine("Ready for START.");
    }

    @Override
    public final void run() {
        beforeDriveRun();

        // The scheduler updates the flywheel controller.
        super.run();

        boolean splitterHeld = controls.isSplitterHeld();
        if (splitterHeld && !splitterButtonWasHeld && !intake.isSplitterMoving()) {
            boolean started = splitterDeployed ? intake.retractSplitter() : intake.deploySplitter();
            if (started) {
                splitterDeployed = !splitterDeployed;
            }
        }
        splitterButtonWasHeld = splitterHeld;

        boolean fireHeld = controls.isFireHeld();
        if (controls.isSpinUpHeld() || fireHeld) {
            shooter.shoot();
            aimTurret(turret);
        } else {
            shooter.stop();
            turret.center();
        }

        // Feed only while shooting is requested and the flywheel is fast enough.
        boolean gateOpen = fireHeld && shooter.canOpenGate();

        if (gateOpen) {
            gate.open();
            intake.runIntakeForShooting();
            shooter.runPreshooter();
        } else {
            if (controls.isSpinUpHeld() || fireHeld) {
                shooter.stopPreshooter();
            }
            gate.close();
            if (!fireHeld && controls.isIntakeHeld()) {
                intake.runIntake();
            } else if (!fireHeld && controls.isOuttakeHeld()) {
                intake.runOuttake();
            } else {
                intake.stop();
            }
        }

        addDriveTelemetry();
        telemetry.addData("Shooter Left Power Actual", shooter.getLeftPower());
        telemetry.addData("Shooter Right Power Actual", shooter.getRightPower());
        telemetry.addData("Shooter Velocity Command", shooter.getCommandedVelocity());
        telemetry.addData("Shooter Left Velocity Actual", shooter.getLeftVelocity());
        telemetry.addData("Shooter Right Velocity Actual", shooter.getRightVelocity());
        telemetry.addData("Shooter Encoder Mismatch", shooter.isEncoderMismatch());
        telemetry.addData("Turret angle command (deg)", turret.getCommandedAngleDegrees());
        telemetry.addData("Intake Power Command", intake.getCommandedPower());
        telemetry.addData("Intake Power Actual", intake.getPower());
        telemetry.addData("Splitter rotation command", splitterDeployed ? "deploy" : "retract / initially stopped");
        telemetry.addData("Splitter timer active", intake.isSplitterMoving());
        telemetry.addData("Fire", !fireHeld ? "-"
                : !gateOpen ? "waiting for shooter speed" : "gate open, feeding");
        telemetry.addData("Gate Position Command", gate.getPosition());
        telemetry.update();
    }

    protected void beforeDriveRun() {
    }

    protected abstract void aimTurret(Turret turret);

    protected abstract void stopDrive();

    protected abstract void addDriveTelemetry();

    @Override
    public final void end() {
        CommandScheduler.getInstance().cancelAll();
        stopDrive();
        shooter.stop();
        intake.stop();
        intake.stopSplitter();
        gate.close();
    }
}
