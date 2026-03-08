package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import frc.robot.Ports;

public class Leds extends SubsystemBase {
    private final CANdle candle;
    private final int totalLeds = 300;
    private final int underglowCount = 150;
    private final int flairCount = 150;
    private final int flairOffset = 150;

    public Leds() {
        candle = new CANdle(Ports.kCANdle, Ports.kCANivoreCANBus);
        
        CANdleConfiguration config = new CANdleConfiguration();
        candle.getConfigurator().apply(config);
    }

    public void setAllianceColor() {
        candle.setControl(new EmptyAnimation(0));
        candle.setControl(new EmptyAnimation(1));
        
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            candle.setControl(new SolidColor(0, totalLeds - 1).withColor(new RGBWColor(255, 0, 0, 0)));
        } else {
            candle.setControl(new SolidColor(0, totalLeds - 1).withColor(new RGBWColor(0, 0, 255, 0)));
        }
    }

    public void setPoliceMode() {
        candle.setControl(new EmptyAnimation(0));
        candle.setControl(new EmptyAnimation(1));
        
        candle.setControl(new SolidColor(flairOffset, flairOffset + flairCount - 1).withColor(new RGBWColor(0, 0, 0, 0)));
        
        candle.setControl(new StrobeAnimation(flairOffset, flairOffset + (flairCount / 2) - 1)
            .withColor(new RGBWColor(255, 0, 0, 0))
            .withSlot(0));
            
        candle.setControl(new StrobeAnimation(flairOffset + (flairCount / 2), flairOffset + flairCount - 1)
            .withColor(new RGBWColor(0, 0, 255, 0))
            .withSlot(1));
    }

    public void setGreen() {
        candle.setControl(new EmptyAnimation(0));
        candle.setControl(new EmptyAnimation(1));
        
        candle.setControl(new SolidColor(0, totalLeds - 1).withColor(new RGBWColor(0, 255, 0, 0)));
    }
}