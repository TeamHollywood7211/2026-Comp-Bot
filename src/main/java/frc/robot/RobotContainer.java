package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.Constants.Driving;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.*;
import frc.util.Elastic;
import frc.util.SwerveTelemetry;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Floor floor = new Floor();
    private final Feeder feeder = new Feeder();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Hanger hanger = new Hanger();
    private final Limelight limelight = new Limelight(Ports.kLimeLightShooter);
    private final Music music = new Music(swerve);
    
    @SuppressWarnings("unused")
    private final GamePhaseSubsystem gamePhase = new GamePhaseSubsystem();
    
    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond));
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final AutoRoutines autoRoutines;

    private double manualRPM = 3000.0;
    private double manualHoodPos = 0.2; 

    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(0.1).withRotationalDeadband(0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SubsystemCommands subsystemCommands = new SubsystemCommands(
            swerve, intake, floor, feeder, shooter, hood, hanger, music,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX());

    public RobotContainer() {
        autoRoutines = new AutoRoutines(swerve, intake, floor, feeder, shooter, hood, hanger, limelight, music);

        LimelightHelpers.setLimelightNTDouble(Ports.kLimeLightShooter, "stream", 1);
        
        SmartDashboard.putStringArray("CameraPublisher/Limelight/streams", 
            new String[]{"mjpeg:http://10.17.76.11:5800"}); 

        Elastic.sendNotification(
            new Elastic.Notification(
                Elastic.NotificationLevel.INFO, 
                "System Online", 
                "MK5n Configuration Loaded")
        );

        configureBindings();
        autoRoutines.configure();
        swerve.registerTelemetry(swerveTelemetry::telemeterize);
    }

    private void configureBindings() {
        configureManualDriveBindings();

        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
                .onTrue(intake.homingCommand())
                .onTrue(hanger.homingCommand());

        driver.rightTrigger().whileTrue(intake.intakeCommand());
        driver.rightBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));
        driver.y().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        driver.a().onTrue(hanger.positionCommand(Hanger.Position.HUNG));

        operator.povUp().onTrue(Commands.runOnce(() -> manualRPM += 250));
        operator.povDown().onTrue(Commands.runOnce(() -> manualRPM = Math.max(0, manualRPM - 250)));
        operator.povRight().onTrue(Commands.runOnce(() -> manualHoodPos = Math.min(0.77, manualHoodPos + 0.05)));
        operator.povLeft().onTrue(Commands.runOnce(() -> manualHoodPos = Math.max(0.01, manualHoodPos - 0.05)));

        operator.leftTrigger().whileTrue(
            Commands.parallel(
                shooter.runShooterCommand(() -> manualRPM),
                Commands.run(() -> hood.setPosition(manualHoodPos), hood).finallyDo(hood::stop)
            ).alongWith(
                Commands.sequence(
                    Commands.waitUntil(() -> shooter.isAtVelocity(manualRPM) && hood.isPositionWithinTolerance()),
                    Commands.parallel(
                        feeder.feedCommand(),
                        floor.runEnd(() -> floor.set(Floor.Speed.FEED), () -> floor.set(Floor.Speed.STOP)),
                        intake.agitateCommand()
                    )
                )
            ).alongWith(
                Commands.startEnd(
                    () -> { if(shooter.isAtVelocity(manualRPM)) operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0); },
                    () -> operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0)
                )
            )
        );

        operator.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
        
        operator.a().onTrue(music.runOnce(() -> music.playSong("cali_girls.chrp")));
        operator.b().onTrue(music.runOnce(music::stop));
    }

    private void configureManualDriveBindings() {
        swerve.setDefaultCommand(
                swerve.applyRequest(() -> m_driveRequest
                        .withVelocityX(-driver.getLeftY() * Driving.kMaxSpeed.in(MetersPerSecond))
                        .withVelocityY(-driver.getLeftX() * Driving.kMaxSpeed.in(MetersPerSecond))
                        .withRotationalRate(-driver.getRightX() * 6.28)));

        driver.back().onTrue(swerve.runOnce(swerve::seedFieldCentric));
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Shooter/Manual Target RPM", manualRPM);
        SmartDashboard.putNumber("Shooter/Manual Target Hood", manualHoodPos);
    }

    public Command getAutonomousCommand() {
        return autoRoutines.getSelectedAuto();
    }
}