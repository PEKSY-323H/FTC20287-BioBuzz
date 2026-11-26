package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.autos.Points.Blue;
import org.firstinspires.ftc.teamcode.utils.Alliance;

/**
 * Minimum BLUE routine, the RED Least Auto rotated to the BLUE side: drive from
 * {@link Points.Blue#START_2} to shooting position 2 ({@link Points.Blue#SHOOT_2}), shoot
 * the 4 preloaded POLLEN, then PARK at {@link Points.Blue#STOP_2} in the BLUE LOADING ZONE
 * (LEAVE 3 + PARK 5).
 *
 * <p>For cooperation with the partner on the other side, WAIT_BEFORE_SHOOT_SECONDS holds at shoot 2
 * until the HIVE has tipped, and WAIT_BEFORE_PARK_SECONDS holds after shooting before parking.
 *
 * <p>Everything before the park drive is cut off PARK_RESERVE_SECONDS before the end so PARK
 * always gets its time.
 */
@Configurable
@Autonomous(name = "Blue Least Auto", group = "Blue", preselectTeleOp = "Field-Centric")
public final class BlueLeastAuto extends BaseAuto {
    // ---------- Cooperation timing: set these in Panels before INIT ----------
    // Seconds to wait at shooting position 2 before shooting, e.g. for the partner to tip the HIVE.
    public static double WAIT_BEFORE_SHOOT_SECONDS = 1.5;
    // Seconds to wait after shooting before driving to PARK, e.g. to stay out of the partner's way.
    public static double WAIT_BEFORE_PARK_SECONDS = 0;

    public static double SHOOT_SECONDS = 3.0;
    // Whatever is still running (waits included) is cut off this long before the end of AUTO,
    // so the drive to PARK always gets its time.
    public static double PARK_RESERVE_SECONDS = 4.0;

    public BlueLeastAuto() {
        super(Alliance.BLUE);
    }

    @Override
    protected Pose startPose() {
        return bluePose(Blue.START_2);
    }

    @Override
    protected Command buildRoutine() {
        Pose start = startPose();
        Pose shoot2 = bluePose(Blue.SHOOT_2);
        Pose park = bluePose(Blue.STOP_2);

        // START_2 defaults to -90 and SHOOT_2 to 270: the same physical orientation.
        Path startToShoot2 = line(start, shoot2);
        // Turns from the shooting heading to STOP_2's heading on the way in.
        Path shoot2ToPark = line(shoot2, park);

        return new SequentialCommandGroup(
                untilSecondsLeft(new SequentialCommandGroup(
                        intakeSlow(),
                        spinUp(),
                        aimWhileShooting(shoot2,
                                follow(startToShoot2),
                                settleAt(shoot2),
                                waitSeconds(WAIT_BEFORE_SHOOT_SECONDS),
                                shoot(SHOOT_SECONDS)),
                        waitSeconds(WAIT_BEFORE_PARK_SECONDS)), PARK_RESERVE_SECONDS),
                // If the cutoff hit before shoot() ran, the flywheel is still spinning.
                new InstantCommand(shooter::stop, shooter),
                intakeOff(),
                retractSplitter(),
                follow(shoot2ToPark)) {
            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                // Stop the intake even if the routine was interrupted during collection.
                shooter.stop();
                intake.stop();
            }
        };
    }

    private static Command waitSeconds(double seconds) {
        return new WaitCommand((long) (Math.max(0.0, seconds) * 1000));
    }
}
