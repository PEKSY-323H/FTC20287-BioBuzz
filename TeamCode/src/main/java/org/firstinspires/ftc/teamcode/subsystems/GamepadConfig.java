package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.jetbrains.annotations.NotNull;

/** Shared control mappings; reads the current gamepad state on every call. */
public final class GamepadConfig {
    public static final double DRIVE_DEADBAND = 0.05;
    // Translation input multiplier for the left stick: 1.0 uses full power. The chassis is
    // light, so full power is hard to control. 0.7 is a starting value, not tuned.
    public static final double DRIVE_LIMIT = 0.7;
    // Turn input multiplier: 0.0 disables turning, 1.0 uses full turn power.
    public static final double TURN_LIMIT = 0.6;
    private final Gamepad gamepad;

    public GamepadConfig(@NotNull final Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public double getForward() {
        return -gamepad.left_stick_y;
    }

    public double getStrafe() {
        return gamepad.left_stick_x;
    }

    public double getTurn() {
        return gamepad.right_stick_x;
    }

    public boolean isIntakeHeld() {
        return gamepad.left_bumper;
    }

    public boolean isOuttakeHeld() {
        return gamepad.left_trigger_pressed;
    }

    public boolean isShootHeld() {
        return gamepad.right_bumper;
    }

    /** Hold to let auto-aim steer the chassis (Field-Centric only). */
    public boolean isAimHeld() {
        return gamepad.right_trigger_pressed;
    }

    /** BACK / SHARE: makes the robot's current facing the field-centric forward. */
    public boolean isResetHeadingHeld() {
        return gamepad.back;
    }

    /** INIT only: X (blue button) selects the BLUE alliance. */
    public boolean isBlueAllianceSelected() {
        return gamepad.x;
    }

    /** INIT only: B (red button) selects the RED alliance. */
    public boolean isRedAllianceSelected() {
        return gamepad.b;
    }

    public void rumble(int durationMs) {
        gamepad.rumble(durationMs);
    }
}
