package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.util.Elastic;
import frc.util.Elastic.NotificationLevel;

public class GamePhaseSubsystem extends SubsystemBase {
    
    private boolean endgameAlertSent = false;
    private boolean allianceShiftAlertSent = false;
    private String currentPhase = "DISABLED";
    private boolean isHubActive = false; 
    private Double switchTime = 0.0;

    // --- SIMULATION CONTROLS ---
    private final SendableChooser<String> simAutoWinner = new SendableChooser<>();
    private final SendableChooser<Alliance> simAlliance = new SendableChooser<>();

    public GamePhaseSubsystem() {
        SmartDashboard.putString("Match/Phase", "DISABLED");
        SmartDashboard.putNumber("Match/Time", 0);
        SmartDashboard.putBoolean("Match/HubActive", false);
        SmartDashboard.putNumber("Robot/Voltage", 0.0); 
        SmartDashboard.putNumber("Match/TimeToSwitch", 0.0); 
        
        // RENAMED: Initialize with waiting text
        SmartDashboard.putString("Match/Who won Auto?", "WAITING FOR FMS...");
        
        if (RobotBase.isSimulation()) {
            simAutoWinner.setDefaultOption("Sim: Red Won Auto", "R");
            simAutoWinner.addOption("Sim: Blue Won Auto", "B");
            simAutoWinner.addOption("Sim: No Winner Yet", "");
            SmartDashboard.putData("Sim Controls/Auto Winner", simAutoWinner);

            simAlliance.setDefaultOption("Sim: Blue Alliance", Alliance.Blue);
            simAlliance.addOption("Sim: Red Alliance", Alliance.Red);
            SmartDashboard.putData("Sim Controls/Alliance", simAlliance);
        }
    }

    @Override
    public void periodic() {
        // --- 1. SIMULATION SETUP ---
        if (RobotBase.isSimulation()) {
            String simWinner = simAutoWinner.getSelected();
            if (simWinner != null) DriverStationSim.setGameSpecificMessage(simWinner);
            
            Alliance selectedAlliance = simAlliance.getSelected();
            if (selectedAlliance == Alliance.Blue) DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
            else DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        }

        // --- 2. UPDATE DASHBOARD DATA ---
        SmartDashboard.putNumber("Robot/Voltage", RobotController.getBatteryVoltage());

        double matchTime = DriverStation.getMatchTime(); 
        boolean isAuto = DriverStation.isAutonomous();
        boolean isTeleop = DriverStation.isTeleopEnabled();
        
        // --- 3. LOGIC ---
        if (DriverStation.isDisabled()) {
            isHubActive = false;
            currentPhase = "DISABLED";
            resetFlags();
        } else {
            checkHubStatus(matchTime, isAuto, isTeleop);
            
            if (isAuto) {
                currentPhase = "AUTO";
            } else if (isTeleop) {
                if (matchTime <= 0.1 && matchTime > -1) {
                    currentPhase = "TELEOP START"; 
                }
                else if (matchTime <= 30) {
                    currentPhase = "ENDGAME";
                    if (!endgameAlertSent && matchTime > 0) {
                        Elastic.sendNotification(new Elastic.Notification(NotificationLevel.ERROR, "END GAME!", "CLIMB NOW!"));
                        endgameAlertSent = true;
                    }
                } 
                else if (matchTime <= 130) { 
                    currentPhase = "ALLIANCE SHIFTS";
                    if (matchTime <= 130 && matchTime > 129 && !allianceShiftAlertSent) {
                         Elastic.sendNotification(new Elastic.Notification(NotificationLevel.WARNING, "SHIFTS STARTED", "Check Hub Status!"));
                        allianceShiftAlertSent = true;
                    }
                } else {
                    currentPhase = "TRANSITION";
                }
            }
        }

        SmartDashboard.putString("Match/Phase", currentPhase);
        SmartDashboard.putNumber("Match/Time", Math.max(0, matchTime));
        SmartDashboard.putBoolean("Match/HubActive", isHubActive);
        SmartDashboard.putNumber("Match/TimeToSwitch",switchTime); 
    }

    private void checkHubStatus(double time, boolean isAuto, boolean isTeleop) {
        String gameData = DriverStation.getGameSpecificMessage();
        Optional<Alliance> myAlliance = DriverStation.getAlliance();

        // --- UPDATED DISPLAY LOGIC ---
        // We calculate the text HERE so it updates every loop, even if we return early later.
        String winnerText = "WAITING FOR FMS...";
        if (gameData != null && !gameData.isEmpty()) {
            if (gameData.contains("R")) winnerText = "RED WON AUTO";
            else if (gameData.contains("B")) winnerText = "BLUE WON AUTO";
            else winnerText = "UNKNOWN DATA: " + gameData;
        }
        // Push to Dashboard immediately
        SmartDashboard.putString("Match/Who won Auto?", winnerText);
        // -----------------------------

        // 1. Safe States (Auto / Transition / EndGame)
        if (isAuto) { isHubActive = true; return; }
        if (isTeleop && (time > 130 || time <= 30)) { isHubActive = true; return; }

        // 2. Data Check
        if (gameData == null || gameData.isEmpty() || myAlliance.isEmpty()) {
            isHubActive = true; return;
        }

        // 3. Shift Logic
        boolean didRedWinAuto = gameData.contains("R");
        boolean amIRed = (myAlliance.get() == Alliance.Red);
        boolean myAllianceWonAuto = (didRedWinAuto == amIRed);

        if      (time <= 130 && time > 105)  { isHubActive = !myAllianceWonAuto; switchTime = time - 105.0; }
        else if (time <= 105 && time >  80)  { isHubActive = myAllianceWonAuto;  switchTime = time - 80.0;  }
        else if (time <= 80  && time >  55)  { isHubActive = !myAllianceWonAuto; switchTime = time - 55.0;  }
        else if (time <= 55  && time >  30)  { isHubActive = myAllianceWonAuto;  switchTime = time - 30.0;  }
    }

    private void resetFlags() {
        endgameAlertSent = false;
        allianceShiftAlertSent = false;
    }
}