// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.Constants.Driving;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.*;
import frc.util.SwerveTelemetry;
import frc.util.Elastic;

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
    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond));
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(0.1).withRotationalDeadband(0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final AutoRoutines autoRoutines = new AutoRoutines(
            swerve, intake, floor, feeder, shooter, hood, hanger, limelight, music);

    private final SubsystemCommands subsystemCommands = new SubsystemCommands(
            swerve, intake, floor, feeder, shooter, hood, hanger, music,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX());
        private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        /*try {
            WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        } catch (Exception e) {
            System.out.println("Warning: Could not start Elastic Layout Server: " + e.getMessage());
        }
*/
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        Elastic.sendNotification(
                new Elastic.Notification(
                    Elastic.NotificationLevel.INFO, 
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
        driver.leftBumper().whileTrue(floor.feedCommand());

        // Use the new runShooterCommand from Shooter.java. I've set it to 3000 RPM as an example.
        operator.rightBumper().whileTrue(shooter.runShooterCommand(3000));
        operator.leftTrigger().and(operator.rightBumper()).whileTrue(feeder.feedCommand());
        operator.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());

        // operator.a().onTrue(music.runOnce(() -> music.playSong("cali_girls.chrp")));
        // operator.b().onTrue(music.runOnce(music::stop));
    }

    private void configureManualDriveBindings() {
        swerve.setDefaultCommand(
                swerve.applyRequest(() -> m_driveRequest.withVelocityX(-driver.getLeftY() * 4.5)
                        .withVelocityY(-driver.getLeftX() * 4.5)
                        .withRotationalRate(-driver.getRightX() * 6.28)));

        // driver.a().onTrue(swerve.runOnce(() -> swerve.setOperatorPerspectiveForward(Rotation2d.k180deg)));
        // driver.b().onTrue(swerve.runOnce(() -> swerve.setOperatorPerspectiveForward(Rotation2d.kCW_90deg)));
        // driver.x().onTrue(swerve.runOnce(() -> swerve.setOperatorPerspectiveForward(Rotation2d.kCCW_90deg)));
        // driver.y().onTrue(swerve.runOnce(() -> swerve.setOperatorPerspectiveForward(Rotation2d.kZero)));
        driver.back().onTrue(swerve.runOnce(swerve::seedFieldCentric));
    }

    public Command getAutonomousCommand() {
        /*return autoRoutines.getSelectedAuto();
        */
        return autoChooser.getSelected();
    }
}