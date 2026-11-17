package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/** Toggles a positional servo between 0.5 and 1.0 on each A-button press. */
@TeleOp(name = "TEST - Servo Zero", group = "Testing")
public final class ServoZeroTest extends LinearOpMode {
    // Match this name to the servo in the Robot Controller hardware configuration.
    private static final String SERVO_NAME = "servo1";
    private static final double FIRST_POSITION = 0.5;
    private static final double SECOND_POSITION = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        Servo servo = hardwareMap.get(Servo.class, SERVO_NAME);
        servo.setDirection(Servo.Direction.FORWARD);

        telemetry.addData("Servo", SERVO_NAME);
        telemetry.addLine("START: position 0.5 | Gamepad 1 A: toggle 0.5 / 1.0");
        telemetry.addLine("Use a positional servo; values are not measured angles.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }
        boolean atSecondPosition = false;
        boolean previousA = gamepad1.a;
        double targetPosition = FIRST_POSITION;
        servo.setPosition(targetPosition);
        while (opModeIsActive()) {
            boolean currentA = gamepad1.a;
            // Toggle only on a new press, so holding A does not repeat.
            if (currentA && !previousA) {
                atSecondPosition = !atSecondPosition;
                targetPosition = atSecondPosition ? SECOND_POSITION : FIRST_POSITION;
                servo.setPosition(targetPosition);
            }
            previousA = currentA;

            telemetry.addData("Servo", SERVO_NAME);
            telemetry.addData("Commanded position", targetPosition);
            telemetry.addLine("Press Gamepad 1 A to toggle 0.5 / 1.0");
            telemetry.update();
            idle();
        }
    }
}
