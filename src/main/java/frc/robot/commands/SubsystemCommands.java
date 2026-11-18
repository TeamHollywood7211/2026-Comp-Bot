package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.FrontRange;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Music;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public final class SubsystemCommands {
    private final Swerve swerve;
    private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;
   
public SubsystemCommands(
            Swerve swerve, Intake intake, Floor floor, Feeder feeder,
            Shooter shooter, Hood hood, Hanger hanger, Music music, Leds leds, FrontRange frontRange,
            DoubleSupplier forwardInput, DoubleSupplier leftInput) {
        this.swerve = swerve;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
    }

    public SubsystemCommands(
            Swerve swerve, Intake intake, Floor floor, Feeder feeder,
            Shooter shooter, Hood hood, Hanger hanger, Music music, Leds leds, FrontRange frontRange) {
        this(swerve, intake, floor, feeder, shooter, hood, hanger, music, leds, frontRange, () -> 0, () -> 0);
    }

    public Command ejectJamCommand() {
        return Commands.parallel(
                shooter.spinUpCommand(3200),
                feeder.reverseCommand(),
                floor.reverseCommand());
    }

    public Command feed() {
        return Commands.sequence(
                // Start feeding only once the shooter is at its requested speed (tight tolerance).
                // These checks have no subsystem requirements, so they won't cancel the shooter.
                Commands.waitUntil(shooter::isVelocityWithinTolerance),
                Commands.waitSeconds(0.25),
                // Jam protection: keep feeding only while the shooter stays above the minimum
                // fraction of its target RPM (looser, so the normal dip while shooting is OK).
                Commands.parallel(
                        feeder.feedCommand(),
                        Commands.waitSeconds(0.125)
                                .andThen(floor.feedCommand().alongWith(intake.agitateCommand())))
                        .onlyWhile(shooter::isAboveFeedMinimum))
                // If the shooter drops too low the feed stops; while the button is still held,
                // start over and wait for the shooter to recover before feeding again.
                .repeatedly();
    }

    public Command aimAndShoot() {
        final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood,
                () -> swerve.getState().Pose);
        return Commands.parallel(
                aimAndDriveCommand,
                Commands.waitSeconds(0.25)
                        .andThen(prepareShotCommand),
                Commands.waitUntil(() -> aimAndDriveCommand.isAimed() && prepareShotCommand.isReadyToShoot())
                        .andThen(feed()));
    }

    public Command shootManually() {
        return shooter.dashboardSpinUpCommand()
                .andThen(feed())
                .handleInterrupt(() -> shooter.stop());
    }

    private double calculatePassingRPM() {
        // Gets current distance to hub in meters
        double distance = swerve.getDistanceToHub();

        // --- TUNE THESE VALUES ON THE FIELD ---
        double minDistance = 3.0; // Meters (Closer to hub)
        double maxDistance = 8.0; // Meters (Mid-field)
        double minRpm = 3500.0; // Slower lob for when you are close
        double maxRpm = 6000.0; // Faster lob for when you are far

        // Clamp distance so we don't calculate RPMs outside our bounds
        double clampedDistance = Math.max(minDistance, Math.min(maxDistance, distance));

        // Map the distance to the RPM range (Linear Interpolation)
        double percentage = (clampedDistance - minDistance) / (maxDistance - minDistance);
        double targetRpm = minRpm + (percentage * (maxRpm - minRpm));

        SmartDashboard.putNumber("Shooter/Passing Target RPM", targetRpm);

        return targetRpm;
    }

    public Command automaticPassingShot() {
        return Commands.parallel(
                // 1. Constantly recalculate and set the RPM based on moving distance
                shooter.runShooterCommand(this::calculatePassingRPM),

                // 2. Lock the hood all the way up for the maximum lob arc
                Commands.run(() -> hood.setPosition(0.77), hood));
        // Feeding is intentionally NOT part of this command. The operator feeds with the
        // right bumper (feed()). If this group also required feeder/floor/intake, pressing
        // right bumper would cancel the whole group (including the shooter spin-up).
    }
}