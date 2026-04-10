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
    private final Leds leds;
    private final FrontRange frontRange;

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
        this.leds = leds;
        this.frontRange = frontRange;
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
                Commands.waitSeconds(0.25),
                Commands.parallel(
                        feeder.feedCommand(),
                        Commands.waitSeconds(0.125)
                                .andThen(floor.feedCommand().alongWith(intake.agitateCommand()))));
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
                Commands.run(() -> hood.setPosition(0.77), hood),

                // 3. Wait for the shooter to hit the moving target, then automatically feed
                Commands.waitUntil(shooter::isVelocityWithinTolerance)
                        .andThen(feed()));
    }
}