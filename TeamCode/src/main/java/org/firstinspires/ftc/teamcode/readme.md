# FTC-20827 BioBuzz TeamCode

Team-owned FTC code for BIOBUZZ. The command architecture uses SolversLib's official command system (`org.solverslib:core:0.3.6`, plus `org.solverslib:pedroPathing:0.3.6` for Pedro Pathing 3 commands in autonomous), following its subsystem, command, and `CommandOpMode` lifecycle model.

Compiling proves the code builds. It does not prove deployment, sensor calibration or robot behavior: every item marked "verify on the robot" below still has to be checked on the real robot.

## Source layout

- `teleops/`: `BaseDriveTeleOp` holds the shared hardware, controls and lifecycle; `RobotCentricDrive` and `FieldCentricDrive` are the Driver Station entries.
- `autos/`: `BaseAuto`, the shared lifecycle and helpers for Pedro autonomous routines, and `RedMainAuto`, the RED autonomous.
- `subsystems/`: hardware ownership. `Drive` (four drive motors), `Shooter`, `Intake`, `Gate`, `Localizer` (OctoQuad heading), `PathFollower` (Pedro follower), plus `GamepadConfig` for the control mapping.
- `commands/`: `DriveCommand`, the drivetrain default command for both TeleOps.
- `pedro/`: `Constants`, the Pedro Pathing 3 follower configuration; `Tuning`, which registers Pedro's AutoTune procedures; `procedures/`, unmodified copies of the four Quickstart procedures this robot needs.
- `utils/`: `Alliance` (field frame, alliance conversion, AprilTag IDs) and `AutoHandoff` (AUTO result for the next TeleOp).
- `tests/`: Driver Station diagnostic OpModes.

## Hardware names

Motors `frontLeft`, `frontRight`, `backLeft`, `backRight`, `shooterLeft`, `shooterRight`, `intake`; servo `gate`; OctoQuad MK2 `octoquad` (type OctoQuadFTC).

The drive motor names and directions live in one place, the constants at the top of `Drive`. `pedro/Constants` reads the same constants, so the TeleOps and the Pedro follower always agree. Positive power on every wheel must roll the robot forward.

## Controls (gamepad 1)

| Input | Action |
| --- | --- |
| Left stick | Translate (scaled by `GamepadConfig.DRIVE_LIMIT`) |
| Right stick X | Turn (scaled by `GamepadConfig.TURN_LIMIT`) |
| LB (hold) | Intake. The gate opens only while LB is held and the shooter is at speed |
| LT (hold) | Outtake |
| RB (hold) | Shooter |
| BACK / SHARE | Make the current facing field-centric forward (Field-Centric only) |

## Field-Centric TeleOp

The left stick translates relative to a fixed forward direction, however the robot is turned. After a 90 degree counterclockwise turn, stick-left still moves the robot to the driver's left.

The forward direction comes from one of three places:

1. Straight after an AUTO (within 30 s), from the AUTO's final pose and alliance: stick-forward is away from the drivers. This assumes the robot was not moved between AUTO and the TeleOp's INIT.
2. Otherwise, the robot's facing at START. Point the front away from the drivers before START.
3. BACK re-zeroes it to the current facing at any time.

Keep the robot still during INIT while the OctoQuad calibrates, and wait for `RUNNING`.

A single bad OctoQuad packet reuses the last good heading for up to 0.1 s (`Localizer.HEADING_HOLD_SECONDS`). If the heading stays unavailable, the sticks become robot-centric instead of stopping the robot, the gamepad rumbles, and telemetry shows `ROBOT-CENTRIC: heading unavailable`. Field-centric resumes when the heading returns.

## Autonomous base

`BaseAuto` is written against the BIOBUZZ Competition Manual TU08. Subclasses pass an `Alliance`, implement `startPose()` and `buildRoutine()`, and write every pose for the BLUE side with `pose(x, y, headingDegrees)`.

Field frame (a team convention, documented in `utils/Alliance`): inches, origin at the audience-side corner next to the RED alliance area, +x toward the BLUE alliance wall, +y away from the audience, heading 0 along +x and counterclockwise-positive. BLUE owns x > 72. RED is the BLUE routine rotated 180 degrees about the field centre, because the field is rotationally symmetric (LOADING ZONES on tiles A5 and F2). Check the frame against a real field before trusting any pose.

What the base does for the rules:

- G304: nothing drives before START; the start pose is re-applied at START in case the robot was nudged. Place the robot on its own side, touching the wall, outside the LOADING ZONE, with 4 POLLEN.
- G402: a pose within `OWN_SIDE_MARGIN_IN` of the centre line shows a warning on the Driver Station.
- G403: everything stops `END_GUARD_SECONDS` before the 30 s mark and again when the OpMode ends.
- LEAVE 3, PARK 5, HIVE TIP 20: `untilSecondsLeft(command, seconds)` interrupts scoring so the final LEAVE / PARK move always gets its time.
- `shoot(feedSeconds)` spins up, opens the gate and feeds, and stops intake, gate and flywheel even when interrupted.

Before any path is driven, `pedro/Constants` still needs real values: the OctoQuad pod ports, directions and offsets are placeholders, and the Foresight block is Pedro's documentation sample. See the next section.

## Red Main Auto

`autos/RedMainAuto` (Driver Station: **Red Main Auto**) drives the eleven paths of `Auto routines/Auto red v1。1.pp.json` in order: an opening line from the wall to the shooting position, then the ten called Path 1-10 below. Its coordinates are the RED-side numbers from that file; a private `red(...)` helper converts them to the BLUE poses `BaseAuto` expects.

It shoots the preload at the shooting position before Path 1 and shoots again after Paths 3, 7 and 9 (`SHOOT_SECONDS` of feeding each), runs the intake along Path 2, and collects at a FLOWER at the end of Paths 5 and 8 (`FLOWER_SECONDS` stopped with the intake on). The FLOWER servo calls are empty placeholders. Everything before the last path is cut off `PARK_RESERVE_SECONDS` before the end so the drive into the RED LOADING ZONE always gets its time.

It has not been run on the robot, and at the default path speed the plan is longer than 30 s, so its end is cut off.

## Pedro tuning (AutoTune)

Deploy, connect to the Robot Controller's Wi-Fi and open `http://192.168.43.1:10158`. Run the procedures in this order, pasting each generated block over the matching one in `pedro/Constants` and redeploying before the next step:

1. **Mecanum Tuner**, wheels off the ground. It confirms motor names and directions. The result must equal the constants in `Drive`; if a wheel is wrong, fix it in `Drive`.
2. **OctoQuad Tuner**, pushed by hand with no motor power. Produces `localizerConfig`.
3. **Tests, Odometry Test**. The robot drives itself forward, left and counterclockwise at half power. Do not continue until it passes.
4. **Foresight Tuner**. The robot drives and spins at full power and coasts past its target: clear floor, one person on the Driver Station STOP. Produces `foresightConfig`.
5. **Tests, Hold / Line / Curve**, to check the finished follower.

The generated blocks declare `public static` fields with the same names as the existing ones, so they replace them directly. Keep `c.manualBrakeMode.set(true)` in `drivetrainConfig` and keep the `Drive.*` constants rather than the literal names AutoTune prints.

## Sloth hot reload

[Sloth](https://github.com/Dairy-Foundation/Sloth) `0.3.2` sends only the `org.firstinspires.ftc.teamcode` classes to the robot, in about a second, and applies them when the running OpMode ends.

- First do one normal install (the **TeamCode** run configuration) so the robot has the Sloth runtime.
- After that, run the **deploySloth** run configuration (`.run/deploySloth.run.xml`), or `./gradlew :TeamCode:deploySloth`, for code changes inside TeamCode.
- Do a normal install again after changing a dependency or anything outside TeamCode. Run `./gradlew :TeamCode:removeSlothRemote` first, or add it as a "before launch" Gradle task of the TeamCode run configuration; otherwise the hot-loaded code still on the robot replaces the newly installed code.

Panels `1.0.20` and Pedro tuning `1.0.3` depend on this same Sloth version. Not tried on the robot yet.

## Dependencies

SolversLib `0.3.6`, Pedro Pathing `revhub:3.0.2` and `tuning:1.0.3` (the versions the official Quickstart pins; 3.0.2 fixes the OctoQuad pod offsets being applied to the wrong axes in 3.0.0), FTC SDK `12.0.0`. SolversLib's `pedroPathing:0.3.6` was built against `revhub:3.0.0`; every Pedro method it calls still exists in 3.0.2. `TeamCode/build.gradle` excludes only SolversLib's transitive `org.firstinspires.ftc` SDK artifacts because its strict 11.1 constraints conflict with the application's explicit SDK 12.0 dependencies.

## Reference

- [SolversLib command system](https://docs.seattlesolvers.com/command-base/command-system)
- [SolversLib CommandOpMode](https://docs.seattlesolvers.com/command-base/command-system/robot-and-commandopmode)
- [SolversLib installation](https://docs.seattlesolvers.com/installation)
- [Pedro Pathing 3 constants](https://pedropathing.com/docs/pathing/tuning/constants) and [OctoQuad localizer](https://pedropathing.com/docs/pathing/tuning/localization/octoquad)
- [BIOBUZZ game and season materials](https://ftc-resources.firstinspires.org/ftc/game)
