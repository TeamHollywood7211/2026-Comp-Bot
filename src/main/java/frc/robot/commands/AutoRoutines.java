package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;

public final class AutoRoutines {
    private final Intake intake;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;
    private final Music music;

    private final SubsystemCommands subsystemCommands;
    private final SendableChooser<Command> autoChooser;

    public AutoRoutines(
            Swerve swerve, Intake intake, Floor floor, Feeder feeder,
            Shooter shooter, Hood hood, Hanger hanger, Limelight limelight, Music music) {
        
        this.intake = intake;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;
        this.music = music;

        this.subsystemCommands = new SubsystemCommands(swerve, intake, floor, feeder, shooter, hood, hanger, music);

        registerNamedCommands();
        this.autoChooser = AutoBuilder.buildAutoChooser();
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("Intake", intake.intakeCommand());
        NamedCommands.registerCommand("StowIntake", intake.runOnce(() -> intake.set(Intake.Position.STOWED)));
        NamedCommands.registerCommand("AimAndShoot", subsystemCommands.aimAndShoot().withTimeout(5));
        
        NamedCommands.registerCommand("SpinUp", Commands.parallel(
                shooter.spinUpCommand(2600),
                hood.positionCommand(0.32)));

        NamedCommands.registerCommand("HangReady", hanger.positionCommand(Hanger.Position.HANGING));
        NamedCommands.registerCommand("HangFinish", hanger.positionCommand(Hanger.Position.HUNG));
        NamedCommands.registerCommand("Play Cali Girls", music.runOnce(() -> music.playSong("cali_girls.chrp")));
        NamedCommands.registerCommand("Stop Music", music.runOnce(music::stop));
    }

    public void configure() {
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Hardcoded array to bypass the broken PathPlanner dynamic methods
        String[] commandNames = {
            "Intake", "StowIntake", "AimAndShoot", "SpinUp", 
            "HangReady", "HangFinish", "Play Cali Girls", "Stop Music"
        };
        SmartDashboard.putStringArray("Registered Named Commands", commandNames);
    }

    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }
}