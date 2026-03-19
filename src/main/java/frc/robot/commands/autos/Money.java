// src/main/java/frc/robot/commands/autos/Money.java
package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.SubsystemCommands;

public class Money {
    public static Command create(SubsystemCommands subsystemCommands) {
        try {
            return Commands.sequence(
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Starting to Right Side Shot")),
                Commands.deadline(Commands.waitSeconds(2.0), subsystemCommands.aimAndShootAuto()),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Right Side Shot to Outpost")),
                subsystemCommands.approachStationCommand(),
                Commands.waitSeconds(5.0),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Outpost to Right Side Shot")),
                Commands.deadline(Commands.waitSeconds(7.0), subsystemCommands.aimAndShootAuto())
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load paths for Money Money Java auto", e.getStackTrace());
            return Commands.none();
        }
    }
}