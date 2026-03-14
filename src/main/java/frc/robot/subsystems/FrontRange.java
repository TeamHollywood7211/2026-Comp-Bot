// src/main/java/frc/robot/subsystems/FrontRange.java
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Ports;

public class FrontRange extends SubsystemBase {
    private static final double kHopperOffsetInches = 11.875;
    private static final double kDesiredGapInches = 1.0;
    private static final Distance kTargetDistance = Inches.of(kHopperOffsetInches + kDesiredGapInches);
    private static final Distance kTolerance = Inches.of(1.0); 

    private final CANrange canrange;

    public FrontRange() {
        canrange = new CANrange(Ports.kFrontRange, Ports.kRoboRioCANBus);
        SmartDashboard.putData(this);
    }

    public Distance getDistance() {
        return canrange.getDistance().getValue();
    }

    public boolean isAtTargetDistance() {
        Distance current = getDistance();
        return current.gt(kTargetDistance.minus(kTolerance)) && current.lt(kTargetDistance.plus(kTolerance));
    }

    public boolean isCloseEnough() {
        return getDistance().lt(kTargetDistance.plus(kTolerance));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Distance (Inches)", () -> getDistance().in(Inches), null);
        builder.addBooleanProperty("At Target Distance", this::isAtTargetDistance, null);
        builder.addBooleanProperty("Is Close Enough", this::isCloseEnough, null);
    }
}