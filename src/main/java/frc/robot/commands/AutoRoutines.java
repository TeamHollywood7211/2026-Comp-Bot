package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Music;

public final class AutoRoutines {
    private final Swerve swerve;
    private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;
    private final Limelight limelight;
    private final Music music;

    private final SubsystemCommands subsystemCommands;
    private final SendableChooser<Command> autoChooser;

    public AutoRoutines(
            Swerve swerve,
            Intake intake,
            Floor floor,
            Feeder feeder,
            Shooter shooter,
            Hood hood,
            Hanger hanger,
            Limelight limelight,
            Music music) {
        this.swerve = swerve;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;
        this.limelight = limelight;
        this.music = music;

        this.subsystemCommands = new SubsystemCommands(swerve, intake, floor, feeder, shooter, hood, hanger, music);

        // Register Named Commands for PathPlanner to call during the auto
        registerNamedCommands();

        // PathPlanner AutoBuilder generates the chooser automatically
        this.autoChooser = AutoBuilder.buildAutoChooser();
    }

    private void registerNamedCommands() {
        // These names must match the names you put in the PathPlanner GUI
        NamedCommands.registerCommand("Intake", intake.intakeCommand());
        NamedCommands.registerCommand("StowIntake", intake.runOnce(() -> intake.set(Intake.Position.STOWED)));
        NamedCommands.registerCommand("AimAndShoot", subsystemCommands.aimAndShoot().withTimeout(5));
        NamedCommands.registerCommand("SpinUp", Commands.parallel(
                shooter.spinUpCommand(2600),
                hood.positionCommand(0.32)));
        NamedCommands.registerCommand("HangReady", hanger.positionCommand(Hanger.Position.HANGING));
        NamedCommands.registerCommand("HangFinish", hanger.positionCommand(Hanger.Position.HUNG));
        NamedCommands.registerCommand("Play Cali Girls", music.runOnce(() -> music.playSong("cali_girls.chrp")));
        NamedCommands.registerCommand("Stop Music", music.runOnce(() -> music.stop()));
    }

    public void configure() {
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Used by RobotContainer to get the selected autonomous command.
     */
    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }
}