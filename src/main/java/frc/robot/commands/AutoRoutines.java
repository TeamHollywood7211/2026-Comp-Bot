package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

public final class AutoRoutines {
    private final Intake intake;   
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;
    private final Music music;

    private final SubsystemCommands subsystemCommands;
    private final SendableChooser<Command> autoChooser;

    public AutoRoutines(
            Swerve swerve, Intake intake, Floor floor, Feeder feeder,
            Shooter shooter, Hood hood, Hanger hanger, Music music, Leds leds, FrontRange frontRange) {
        
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;
        this.music = music;

        this.subsystemCommands = new SubsystemCommands(swerve, intake, floor, feeder, shooter, hood, hanger, music, leds, frontRange);

        registerNamedCommands();
        this.autoChooser = AutoBuilder.buildAutoChooser();
    }

    public void registerNamedCommands() {
        NamedCommands.registerCommand("StowIntake", intake.runOnce(() -> intake.set(Intake.Position.STOWED)));
        NamedCommands.registerCommand("ApproachStation", subsystemCommands.approachStationCommand());
        
        NamedCommands.registerCommand("Intake", Commands.runOnce(() -> {
            intake.set(Intake.Position.INTAKE);
            intake.set(Intake.Speed.INTAKE);
        }, intake));

        NamedCommands.registerCommand("IntakeStop", Commands.runOnce(() -> {
            intake.set(Intake.Speed.STOP);
        }, intake));

        NamedCommands.registerCommand("Shoot", Commands.runOnce(() -> {
            shooter.setRPM(3200);
        }, shooter));

        NamedCommands.registerCommand("Feed", Commands.runOnce(() -> {
            feeder.set(Feeder.Speed.FEED);
            floor.set(Floor.Speed.FEED);
        }, feeder, floor));

        NamedCommands.registerCommand("AutoContinuousShoot", subsystemCommands.autoContinuousShootCommand());

        NamedCommands.registerCommand("StopAll", Commands.runOnce(() -> {
            shooter.setRPM(0);
            feeder.stop();
            floor.stop();
            intake.set(Intake.Speed.STOP);
        }, shooter, feeder, floor, intake));

        NamedCommands.registerCommand("SpinUp", Commands.parallel(shooter.spinUpCommand(3200), hood.positionCommand(0.32)));

        NamedCommands.registerCommand("Play Cali Girls", music.runOnce(() -> music.playSong("cali_girls.chrp")));
        NamedCommands.registerCommand("Stop Music", music.runOnce(music::stop));
    }

    public void configure() {
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public SendableChooser<Command> getAutoChooser() {
        return autoChooser;
    }

    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }
}