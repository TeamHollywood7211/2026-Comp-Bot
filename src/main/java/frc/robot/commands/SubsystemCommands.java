package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
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
        DoubleSupplier forwardInput, DoubleSupplier leftInput
    ) {
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
        Shooter shooter, Hood hood, Hanger hanger, Music music, Leds leds, FrontRange frontRange
    ) {
        this(swerve, intake, floor, feeder, shooter, hood, hanger, music, leds, frontRange, () -> 0, () -> 0);
    }

    public Command approachStationCommand() {
        final SwerveRequest.RobotCentric forwardRequest = new SwerveRequest.RobotCentric()
            .withVelocityX(0.75).withVelocityY(0).withRotationalRate(0); 
        final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

        return Commands.sequence(
            swerve.applyRequest(() -> forwardRequest).until(frontRange::isCloseEnough),
            Commands.parallel(
                swerve.applyRequest(() -> brakeRequest).withTimeout(0.2), 
                Commands.runEnd(leds::setGreenFlash, leds::setAllianceColor, leds).withTimeout(3.0)
            )
        );
    }

    public Command aimAndShootTeleop() {
        final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(
            swerve, forwardInput, leftInput
        );
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getState().Pose);
        
        return Commands.parallel(
            aimAndDriveCommand,
            Commands.run(() -> leds.setAimingMode(aimAndDriveCommand.isAimed()), leds).finallyDo(leds::setAllianceColor),
            Commands.waitSeconds(0.25).andThen(prepareShotCommand),
            Commands.waitUntil(() -> aimAndDriveCommand.isAimed() && prepareShotCommand.isReadyToShoot())
                .andThen(feed())
        );
    }

    public Command aimAndShootAuto() {
        final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(
            swerve, () -> 0.0, () -> 0.0
        );
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getState().Pose);
        
        return Commands.parallel(
            aimAndDriveCommand,
            Commands.run(() -> leds.setAimingMode(aimAndDriveCommand.isAimed()), leds).finallyDo(leds::setAllianceColor),
            Commands.waitSeconds(0.25).andThen(prepareShotCommand)
        ).raceWith(
            Commands.waitUntil(() -> aimAndDriveCommand.isAimed() && prepareShotCommand.isReadyToShoot())
                .andThen(autoFeedCommand())
        );
    }

    public Command shootManually() {
        return shooter.dashboardSpinUpCommand()
            .andThen(feed())
            .handleInterrupt(() -> shooter.stop());
    }

    public Command ejectJamCommand() {
        return Commands.parallel(
            shooter.runReverseCommand(),
            feeder.reverseCommand(),
            floor.reverseCommand()
        );
    }

    public Command feed() {
        return Commands.sequence(
            Commands.waitSeconds(0.25),
            Commands.parallel(
                feeder.feedCommand(),
                Commands.waitSeconds(0.125)
                    .andThen(floor.feedCommand().alongWith(intake.agitateCommand()))
            )
        );
    }

    public Command autoFeedCommand() {
        return Commands.parallel(
            feeder.feedCommand(),
            floor.feedCommand()
        );
    }

    // Unified infinite auto shoot command
    public Command autoContinuousShootCommand() {
        double targetRPM = 3200.0;
        return Commands.parallel(
            shooter.runShooterCommand(() -> targetRPM),
            Commands.sequence(
                Commands.waitUntil(() -> shooter.isAtVelocity(targetRPM)),
                Commands.parallel(
                    feeder.feedCommand(),
                    floor.feedCommand(),
                    intake.agitateCommand()
                )
            )
        );
    }
}