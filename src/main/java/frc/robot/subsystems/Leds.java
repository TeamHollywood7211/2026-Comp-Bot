package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import frc.robot.Ports;

public class Leds extends SubsystemBase {
    private final CANdle candle;
    private final int totalLeds = 300;
    private final int flairCount = 150;
    private final int flairOffset = 150;

    private String currentLedState = "NONE";

    public Leds() {
        candle = new CANdle(Ports.kCANdle, Ports.kRoboRioCANBus);
        
        CANdleConfiguration config = new CANdleConfiguration();
        candle.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            setConnectionStatus(DriverStation.isDSAttached());
        }
    }

    public void setAllianceColor() {
        if (currentLedState.equals("ALLIANCE_DEFAULT")) return;
        currentLedState = "ALLIANCE_DEFAULT";

        candle.setControl(new EmptyAnimation(0));
        candle.setControl(new EmptyAnimation(1));
        
        Optional<Alliance> alliance = DriverStation.getAlliance();
        RGBWColor underglowColor = (alliance.isPresent() && alliance.get() == Alliance.Red) 
            ? new RGBWColor(255, 0, 0, 0) 
            : new RGBWColor(0, 0, 255, 0);
            
        candle.setControl(new SolidColor(0, flairOffset - 1).withColor(underglowColor));
        candle.setControl(new RainbowAnimation(flairOffset, flairOffset + flairCount - 1).withSlot(0));
    }

    public void setAimingMode(boolean locked) {
        String stateKey = "AIMING_" + locked;
        if (currentLedState.equals(stateKey)) return; 
        currentLedState = stateKey;

        Optional<Alliance> alliance = DriverStation.getAlliance();
        RGBWColor underglowColor = (alliance.isPresent() && alliance.get() == Alliance.Red) 
            ? new RGBWColor(255, 0, 0, 0) 
            : new RGBWColor(0, 0, 255, 0);

        candle.setControl(new SolidColor(0, flairOffset - 1).withColor(underglowColor));

        if (locked) {
            candle.setControl(new EmptyAnimation(0));
            candle.setControl(new EmptyAnimation(1));
            candle.setControl(new RainbowAnimation(flairOffset, flairOffset + flairCount - 1).withSlot(0));
        } else {
            candle.setControl(new EmptyAnimation(0));
            candle.setControl(new EmptyAnimation(1));
            
            candle.setControl(new StrobeAnimation(flairOffset, flairOffset + (flairCount / 2) - 1)
                .withColor(new RGBWColor(255, 0, 0, 0))
                .withSlot(0));
                
            candle.setControl(new StrobeAnimation(flairOffset + (flairCount / 2), flairOffset + flairCount - 1)
                .withColor(new RGBWColor(0, 0, 255, 0))
                .withSlot(1));
        }
    }

    public void setPoliceMode() {
        if (currentLedState.equals("POLICE")) return;
        currentLedState = "POLICE";

        candle.setControl(new EmptyAnimation(0));
        candle.setControl(new EmptyAnimation(1));
        
        candle.setControl(new SolidColor(flairOffset, totalLeds - 1).withColor(new RGBWColor(0, 0, 0, 0)));
        
        candle.setControl(new StrobeAnimation(flairOffset, flairOffset + (flairCount / 2) - 1)
            .withColor(new RGBWColor(255, 0, 0, 0))
            .withSlot(0));
            
        candle.setControl(new StrobeAnimation(flairOffset + (flairCount / 2), flairOffset + flairCount - 1)
            .withColor(new RGBWColor(0, 0, 255, 0))
            .withSlot(1));
    }

    public void setGreen() {
        if (currentLedState.equals("SOLID_GREEN")) return;
        currentLedState = "SOLID_GREEN";

        candle.setControl(new EmptyAnimation(0));
        candle.setControl(new EmptyAnimation(1));
        
        candle.setControl(new SolidColor(0, totalLeds - 1).withColor(new RGBWColor(0, 255, 0, 0)));
    }

    public void setGreenFlash() {
        if (currentLedState.equals("FLASH_GREEN")) return;
        currentLedState = "FLASH_GREEN";

        candle.setControl(new EmptyAnimation(0));
        candle.setControl(new EmptyAnimation(1));
        
        candle.setControl(new StrobeAnimation(0, totalLeds - 1)
            .withColor(new RGBWColor(0, 255, 0, 0))
            .withSlot(0));
    }

    public void setConnectionStatus(boolean isConnected) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        String stateKey = "CONN_" + isConnected + "_" + alliance.orElse(Alliance.Blue).toString();
        
        if (currentLedState.equals(stateKey)) return; 
        currentLedState = stateKey;

        candle.setControl(new EmptyAnimation(0));
        candle.setControl(new EmptyAnimation(1));

        RGBWColor underglowColor = (alliance.isPresent() && alliance.get() == Alliance.Red) 
            ? new RGBWColor(255, 0, 0, 0) 
            : new RGBWColor(0, 0, 255, 0);
        candle.setControl(new SolidColor(0, flairOffset - 1).withColor(underglowColor));

        if (isConnected) {
            candle.setControl(new SingleFadeAnimation(flairOffset, flairOffset + flairCount - 1)
                .withColor(new RGBWColor(0, 255, 0, 0))
                .withFrameRate(10) 
                .withSlot(0));
        } else {
            candle.setControl(new SolidColor(flairOffset, totalLeds - 1).withColor(new RGBWColor(150, 0, 255, 0)));
        }
    }
}