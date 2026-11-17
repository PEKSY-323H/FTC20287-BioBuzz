# FTC-20827 BioBuzz TeamCode

Team-owned FTC code for BIOBUZZ. The command architecture uses SolversLib's official command system (`org.solverslib:core:0.3.6`, plus `org.solverslib:pedroPathing:0.3.6` for Pedro Pathing 3 commands in autonomous), following its subsystem, command, and `CommandOpMode` lifecycle model.

Compiling proves the code builds. It does not prove deployment, sensor calibration or robot behavior: every item marked "verify on the robot" below still has to be checked on the real robot.

## Source layout

- `teleops/`: `BaseDriveTeleOp` holds the shared hardware, controls and lifecycle; `PedroDriveTeleOp` adds the shared Pedro follower, B/X paths, AUTO handoff and odometry reset; `RobotCentricDrive` and `FieldCentricDrive` are the Driver Station entries.
- `autos/`: `BaseAuto`, shared `Points`, and the Red/Blue Full, Least, Cooperation and Leave routines, plus Blue Shoot 1, Delayed Leave and Cooperation Down Park variants.
- `subsystems/`: hardware ownership. `Drive` (four drive motors), `Shooter`, `Intake`, `Gate`, `Localizer` (OctoQuad heading), `PathFollower` (Pedro follower), plus `GamepadConfig` for the control mapping.
- `commands/`: `DriveCommand`, a plain drivetrain default command (no TeleOp uses it now; both drive through Pedro).
- `pedro/`: `Constants`, the Pedro Pathing 3 follower configuration; `Tuning`, which registers Pedro's AutoTune procedures; `procedures/`, unmodified copies of the four Quickstart procedures this robot needs.
- `utils/`: `Alliance` (field frame, alliance conversion, AprilTag IDs) and `AutoHandoff` (AUTO result for the next TeleOp).
- `tests/`: Driver Station diagnostic OpModes.

## Hardware names

Motors `frontLeft`, `frontRight`, `backLeft`, `backRight`, `shooterLeft`, `shooterRight`, `intake`; positional servo `gate`; continuous-rotation servo `splitter`; OctoQuad MK2 `octoquad` (type OctoQuadFTC).

Configure `splitter` as a Servo in Robot Configuration, with the physical servo in continuous-rotation mode. `Servo.setPosition()` supplies rotation commands, not angles: deploy defaults to 1.0, retract to 0.0, and neutral to 0.5 (direction is FORWARD). Calibrate `Intake.SPLITTER_DEPLOY_POSITION`, `SPLITTER_RETRACT_POSITION`, `SPLITTER_STOP_POSITION`, and the separate `SPLITTER_DEPLOY_SECONDS` / `SPLITTER_RETRACT_SECONDS` travel times in Panels. Deploy/retract times currently default to 0.75/0.76 seconds; measure them for the mechanism. A nonpositive or nonfinite duration disables that movement. Initialization sends neutral. Each accepted A press switches deploy/retract direction and starts its timer; expiry sends neutral without blocking driving. Presses during movement are discarded, requiring a release and a fresh press afterward. Invalid or disabled movements do not toggle the target. OpMode STOP also sends neutral. FLOWER collection uses the deploy timer and sends neutral when finished or interrupted.

The drive motor names and directions live in one place, the constants at the top of `Drive`. `pedro/Constants` reads the same constants, so the TeleOps and the Pedro follower always agree. Positive power on every wheel must roll the robot forward. In the TeleOps, wheel commands use normalized motor power without drive motor encoders. The Pedro follower also sends power directly.

## Drivetrain stages and ownership

Before Pedro follower tuning is complete, use `Drive` with `DriveCommand` for direct manual driving. Its robot-centric constructor needs no localizer. For field-centric driving, also construct `Localizer`, establish its field pose, and use the localizer constructor of `DriveCommand`. The OctoQuad setup and localization still need calibration; this stage does not require Foresight/path-following tuning. Install the command as the drive subsystem's default command in a dedicated `CommandOpMode` or a `BaseDriveTeleOp` subclass. The current `RobotCentricDrive` and `FieldCentricDrive` entries both use Pedro, so they are the later-stage entries rather than ready-made direct-drive entries.

After tuning and verifying Pedro, use its follower for manual drive, path following and localization together. Autonomous wraps it in the `PathFollower` subsystem, whose `periodic()` updates it through the scheduler. `PedroDriveTeleOp` owns the raw follower and updates it explicitly once per running loop. These are intentional lifecycle choices. Never construct `Drive` or standalone `Localizer` alongside a Pedro follower in the same OpMode, because they would claim the same hardware. Do not update the same follower both manually and through subsystem `periodic()`.

## Team coding conventions

- Team-owned Java uses four-space indentation, spaces around operators, opening braces on the same line, braces for every conditional body, and multiline methods/getters. Declare each field separately.
- Use complete `Power` and `Velocity` names in team APIs (`getPower()`, `getCommandedVelocity()`, `SHOOT_VELOCITY`); shooter velocity is encoder ticks/s. `hMap` and `hardwareMap` are both acceptable hardware-map parameter names. Prefer `Alliance` for new alliance parameters.
- Keep hardware operations and continuous controllers in subsystems where applicable, compose autonomous behavior with commands, and keep live TeleOp input handling in the shared TeleOp lifecycle. Diagnostic OpModes may use `LinearOpMode` directly.
- Shared mechanism tuning belongs to its subsystem. Uppercase `public static` tuning fields are mutable Panels settings; `final` distinguishes fixed constants. Record units and when a value is read.
- Preserve the official copies in `pedro/procedures/` without applying team formatting or naming changes. Update them from the pinned upstream source described in `Tuning`.

## Controls

Gamepad 1 (driver):

| Input | Action |
| --- | --- |
| Left stick | Translate (scaled by `GamepadConfig.DRIVE_LIMIT`) |
| Right stick X | Turn (scaled by `GamepadConfig.TURN_LIMIT`) |
| LB (hold) | Intake |
| LT (hold) | Outtake |
| A (press) | Splitter down / up |
| B (press) | Pedro drives to shoot1 |
| X (press) | Pedro drives to shoot2 |
| D-pad down (press) | Reset odometry to the alliance's reset pose |
| D-pad up (press) | Field-Centric only: toggle robot-centric fallback and cancel the current path |

Gamepad 2 (operator):

| Input | Action |
| --- | --- |
| LB (hold) | Intake, same as gamepad 1 |
| LT (hold) | Outtake, same as gamepad 1 |
| RB (hold) | Shooter spin-up |
| RT (hold) | Run shooter; open gate and run shooting intake only while `canOpenGate()` is true |

Intake and outtake work from either gamepad; intake takes priority over outtake. Gamepad 2 RB spins up the shooter only. Gamepad 2 RT also runs the shooter and requests feeding: the gate opens and the intake runs at `Intake.SHOOT_INTAKE_POWER` (default 0.8) only while `canOpenGate()` is true. Otherwise the gate closes and feeding stops. Releasing RT closes the gate; RB may remain held to keep the shooter spinning.

## Field-Centric and Robot-Centric TeleOps

Both run on `PedroDriveTeleOp`: one Pedro follower for manual drive, the B/X paths and the OctoQuad odometry, so the pose, the B/X shooting points, the AUTO handoff and the reset work the same in both, and their settings live in Panels under `PedroDriveTeleOp`. Field-Centric drives relative to the field (below); Robot-Centric drives relative to the robot's facing.

Manual driving and paths use the same field pose as AUTO: inches, with heading in radians and counterclockwise-positive. In Field-Centric with AUTO handoff, stick-forward moves away from the alliance drivers (+x RED, -x BLUE). Without handoff, heading zero (+x) is forward for either alliance. D-pad up toggles a robot-centric fallback; Robot-Centric always follows the robot's facing. Pedro manual inputs are forward-positive, left-positive and CCW-positive; the gamepad adapter converts strafe and turn signs explicitly.

Handoff: when an AUTO ended less than 30 s ago, INIT starts from the odometry pose where that AUTO ended, with its alliance (`USE_AUTO_HANDOFF`). The Driver Station shows that pose during INIT. Re-INITing the TeleOp picks it up again. Otherwise, set `START_X_IN`, `START_Y_IN`, `START_HEADING_DEG`, and `START_ON_BLUE` before INIT. During standalone INIT, X selects BLUE and B selects RED; these presses do not launch paths. Keep the robot still during INIT while Pedro initializes the OctoQuad.

Reset: gamepad 1 D-pad down overwrites the odometry pose with `RED_RESET_X_IN` / `RED_RESET_Y_IN` / `RED_RESET_HEADING_DEG` or the `BLUE_RESET_*` values, by the current alliance, cancels a running B/X path, and rumbles gamepad 1. The reset values are placeholders (0) until measured.

Press B for shoot1 or X for shoot2. Moving a drive stick cancels the path. One Pedro follower handles manual drive, path following, and localization; the standalone `Localizer` subsystem is not constructed in this TeleOp.

## Autonomous base

`BaseAuto` documents the BIOBUZZ Competition Manual TU08 assumptions used by this code. Subclasses pass an `Alliance` and implement `startPose()` and `buildRoutine()`. Current routines read `Points.Red` through `redPose(...)` or `Points.Blue` through `bluePose(...)`. The alternate `pose(x, y, headingDegrees)` helper accepts BLUE-authored coordinates and converts them for the selected alliance; use it only when deliberately following that authoring convention. Avoid converting a field pose twice.

Field frame (a team convention, documented in `utils/Alliance`): inches, origin at the audience-side corner next to the RED alliance area, +x toward the BLUE alliance wall, +y away from the audience, heading 0 along +x and counterclockwise-positive. BLUE owns x > 70.75 (Pedro's field is 141.5 in square, `Alliance.FIELD_SIZE`). RED is the BLUE routine rotated 180 degrees about the field centre, because the field is rotationally symmetric (LOADING ZONES on tiles A5 and F2). Check the frame against a real field before trusting any pose.

What the base does for the rules:

- G304: nothing drives before START; the start pose is re-applied at START in case the robot was nudged. Place the robot on its own side, touching the wall, outside the LOADING ZONE, with 4 POLLEN.
- G402: a pose within `OWN_SIDE_MARGIN_IN` of the centre line shows a warning on the Driver Station.
- G403: everything stops `END_GUARD_SECONDS` before the 30 s mark and again when the OpMode ends.
- LEAVE 3, PARK 5, HIVE TIP 20: `untilSecondsLeft(command, seconds)` interrupts scoring so the final LEAVE / PARK move always gets its time.
- `shoot(feedSeconds)` uses the TeleOp gate/intake condition: it spins up with the gate closed and intake stopped, waits for `canOpenGate()`, then starts a `feedSeconds` shooting window. During that window, readiness is checked every loop; a speed drop closes the gate and stops feeding without extending the window. There is no forced-feed timeout. The parking reserve/end guard can interrupt waiting or feeding, and interruption closes the gate and stops intake and shooter.

The current `pedro/Constants` contains populated localizer and Foresight configuration blocks. Treat the source as the current configuration, and verify it against this robot and the tuning procedure below; the presence of numbers does not establish their calibration or validation status.

## Points and autonomous routines

Prefer one set of RED points in `Points.Red`. `Points.Blue` derives each point by rotating the current RED point 180 degrees about (70.75, 70.75): `(141.5 - x, 141.5 - y, heading + 180 degrees)`. Each read resolves the current RED entry, including replacement through Panels. Auto routines resolve points and build paths during INIT, so re-INIT after edits. TeleOp resolves shooting targets when starting each new path. Copy tuned values back into source to retain them across restarts.

Keep independent alliance tuning available when one point needs a different field value. `Points.Point` implements `PoseSource` and converts degrees to a pose without rotating it. To make one BLUE point independent, replace its derived declaration in `Points.Blue` with a `Point` containing BLUE field coordinates; add `@Configurable` to that class and expose the point as `public static Point` if Panels tuning is needed. Continue passing it through `bluePose(...)`, which checks the own-side margin without rotating it. Other BLUE points can remain derived. The BLUE-authored `pose(...)` / `Alliance.fromBlue(...)` workflow remains available for routines intentionally authored that way.

Red/Blue Full, Least, Cooperation Up, Cooperation Down and Leave are separate Driver Station entries. BLUE also has Shoot 1, Delayed Leave and Cooperation Down Park variants. Read each class's `buildRoutine()` for the actual sequence, timing and heading profiles. Route-editor reference files live in the repository's `auto-pp-json/` directory; Java `Points` and routine definitions determine runtime paths.

Collection uses `Intake.INTAKE_POWER`, loaded transport uses the shared `Intake.INTAKE_SLOW_POWER` via `intakeSlow()`, and shooting feed uses `Intake.SHOOT_INTAKE_POWER`. Keep these parameters in `Intake`, including for Blue Shoot 1. Routine-specific waits, shoot windows and parking reserves stay with the routine. Most scoring routines reserve parking time with `untilSecondsLeft(...)`; `BaseAuto` also stops all mechanisms at its end guard.

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
