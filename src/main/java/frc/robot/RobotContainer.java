package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import java.util.Optional;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.Driving;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.*;
import frc.util.SwerveTelemetry;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;

public class RobotContainer {
    // Manual instantiation avoids factory type mismatch
    private final Swerve swerve = new Swerve();

    private final Intake intake = new Intake();
    private final Floor floor = new Floor();
    private final Feeder feeder = new Feeder();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Hanger hanger = new Hanger();
    private final Limelight limelight = new Limelight("limelight");

    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond));
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(0.1).withRotationalDeadband(0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final AutoRoutines autoRoutines = new AutoRoutines(
            swerve, intake, floor, feeder, shooter, hood, hanger, limelight);

    private final SubsystemCommands subsystemCommands = new SubsystemCommands(
            swerve, intake, floor, feeder, shooter, hood, hanger,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX());

    public RobotContainer() {
        try {
            WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        } catch (Exception e) {
            System.out.println("Warning: Could not start Elastic Layout Server: " + e.getMessage());
        }

        // 3. Send Notification
        Elastic.sendNotification(
                new Elastic.Notification(
                    NotificationLevel.INFO, 
                    "System Online", 
                    "Dual-Controller Mode Active"));
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

        // operator controller yay//
        operator.rightBumper().whileTrue(subsystemCommands.shootManually());
        operator.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
    }

    private void configureManualDriveBindings() {
        swerve.setDefaultCommand(
                swerve.applyRequest(() -> m_driveRequest.withVelocityX(-driver.getLeftY() * 4.5)
                        .withVelocityY(-driver.getLeftX() * 4.5)
                        .withRotationalRate(-driver.getRightX() * 6.28)));

        driver.a().onTrue(swerve.runOnce(() -> swerve.setOperatorPerspectiveForward(Rotation2d.k180deg)));
        driver.b().onTrue(swerve.runOnce(() -> swerve.setOperatorPerspectiveForward(Rotation2d.kCW_90deg)));
        driver.x().onTrue(swerve.runOnce(() -> swerve.setOperatorPerspectiveForward(Rotation2d.kCCW_90deg)));
        driver.y().onTrue(swerve.runOnce(() -> swerve.setOperatorPerspectiveForward(Rotation2d.kZero)));
        driver.back().onTrue(swerve.runOnce(swerve::seedFieldCentric));
    }

    public Command getAutonomousCommand() {
        return autoRoutines.getSelectedAuto();
    }
}