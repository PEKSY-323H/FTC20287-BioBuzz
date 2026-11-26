package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.jetbrains.annotations.NotNull;

/**
 * Shared TeleOp controls; reads the current gamepad 1 state on every call.
 */
public final class GamepadConfig {
    public static final double DRIVE_DEADBAND = 0.05;
    // Translation input multiplier for the left stick: 1.0 uses full power. The chassis is
    // light, so full power is hard to control. 0.7 is a starting value, not tuned.
    public static final double DRIVE_LIMIT = 1.0;
    // Turn input multiplier: 0.0 disables turning, 1.0 uses full turn power.
    public static final double TURN_LIMIT = 0.7;
    private final Gamepad gamepad1;

    public GamepadConfig(@NotNull final Gamepad gamepad1) {
        this.gamepad1 = gamepad1;
    }

    /** LB (hold): intake. Takes priority over outtake. */
    public boolean isIntakeHeld() {
        return gamepad1.left_bumper;
    }

    /** LT (hold): outtake. */
    public boolean isOuttakeHeld() {
        return gamepad1.left_trigger_pressed;
    }

    /** RB (hold): spin up the flywheel. */
    public boolean isSpinUpHeld() {
        return gamepad1.right_bumper;
    }

    /** RT (hold): spin up the flywheel and request feeding at speed. */
    public boolean isFireHeld() {
        return gamepad1.right_trigger_pressed;
    }

    public double getForward() {
        return -gamepad1.left_stick_y;
    }

    public double getStrafe() {
        return gamepad1.left_stick_x;
    }

    public double getTurn() {
        return gamepad1.right_stick_x;
    }

    /** D-pad right (press): switch between field-centric and robot-centric driving. */
    public boolean isDriveModeToggleHeld() {
        return gamepad1.dpad_right;
    }

    /** D-pad up (press): lowers or raises the Flower splitter. */
    public boolean isSplitterHeld() {
        return gamepad1.dpad_up;
    }

    /** Y (press): resets odometry to the alliance's reset pose. */
    public boolean isResetPoseHeld() {
        return gamepad1.y;
    }

    /** B: drive to the first shooting position (Field-Centric only). */
    public boolean isDriveToShoot1Held() {
        return gamepad1.b;
    }

    /** X: drive to the second shooting position (Field-Centric only). */
    public boolean isDriveToShoot2Held() {
        return gamepad1.x;
    }

    /** INIT only: X (blue button) selects the BLUE alliance. */
    public boolean isBlueAllianceSelected() {
        return gamepad1.x;
    }

    /** INIT only: B (red button) selects the RED alliance. */
    public boolean isRedAllianceSelected() {
        return gamepad1.b;
    }

    public void rumble(int durationMs) {
        gamepad1.rumble(durationMs);
    }
}
