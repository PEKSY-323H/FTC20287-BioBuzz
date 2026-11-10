package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.jetbrains.annotations.NotNull;

/** Owns the gate servo; the TeleOp decides when to open or close it. */
@Configurable
public final class Gate extends SubsystemBase {
    // Placeholder positions in [0, 1]; tune for the mechanism in Panels.
    public static double OPEN_POSITION = 0.55;
    public static double CLOSED_POSITION = 0.25;

    private final Servo gate;

    public Gate(@NotNull final HardwareMap hMap) {
        gate = hMap.get(Servo.class, "gate");
        gate.setDirection(Servo.Direction.FORWARD);
        close();
    }

    public void open() {
        setPosition(OPEN_POSITION);
    }

    public void close() {
        setPosition(CLOSED_POSITION);
    }

    public void setPosition(double position) {
        gate.setPosition(Math.max(0.0, Math.min(1.0, position)));
    }

    /** Last commanded position, not a measurement of the servo angle. */
    public double getPosition() {
        return gate.getPosition();
    }
}
