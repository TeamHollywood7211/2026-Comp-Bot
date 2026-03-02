package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class Shooter extends SubsystemBase {
    private static final AngularVelocity kVelocityTolerance = RPM.of(118);

    private final TalonFX leftMotor, middleMotor, rightMotor;
    private final List<TalonFX> motors;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private double dashboardTargetRPM = 0.0;
    private double currentRequestedRPM = 0.0;

    public Shooter() {
        leftMotor = new TalonFX(Ports.kShooterLeft, Ports.kCANivoreCANBus);
        middleMotor = new TalonFX(Ports.kShooterMiddle, Ports.kCANivoreCANBus);
        rightMotor = new TalonFX(Ports.kShooterRight, Ports.kCANivoreCANBus);
        motors = List.of(leftMotor, middleMotor, rightMotor);

        configureMotor(leftMotor, InvertedValue.Clockwise_Positive);
        configureMotor(middleMotor, InvertedValue.CounterClockwise_Positive);
        configureMotor(rightMotor, InvertedValue.CounterClockwise_Positive);

        SmartDashboard.putData(this);
    }

    private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = invertDirection;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Voltage.PeakReverseVoltage = 0.0;
        config.CurrentLimits.StatorCurrentLimit = 120.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 70.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Slot0.kP = 0.5;
        config.Slot0.kI = 2.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond);
        motor.getConfigurator().apply(config);
    }

    public void setRPM(double rpm) {
        this.currentRequestedRPM = rpm;
        for (final TalonFX motor : motors) {
            motor.setControl(velocityRequest.withVelocity(RPM.of(rpm)));
        }
    }

    public void stop() {
        for (final TalonFX motor : motors) {
            motor.setControl(voltageRequest.withOutput(0.0));
        }
        this.currentRequestedRPM = 0.0;
    }

    /** * The primary check for velocity. 
     * @param targetRPM The speed to check against.
     */
    public boolean isAtVelocity(double targetRPM) {
        if (targetRPM <= 0.0) return false;
        return motors.stream().allMatch(motor -> 
            motor.getVelocity().getValue().isNear(RPM.of(targetRPM), kVelocityTolerance)
        );
    }

    /** Helper for commands that don't want to pass a specific RPM */
    public boolean isVelocityWithinTolerance() {
        return isAtVelocity(currentRequestedRPM);
    }

    public Command spinUpCommand(double rpm) {
        return runOnce(() -> setRPM(rpm))
            .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }

    public Command runShooterCommand(double rpm) {
        return runEnd(() -> setRPM(rpm), this::stop);
    }

    public Command runShooterCommand(DoubleSupplier rpmSupplier) {
        return runEnd(() -> setRPM(rpmSupplier.getAsDouble()), this::stop);
    }

    public Command dashboardSpinUpCommand() {
        return run(() -> setRPM(dashboardTargetRPM)).finallyDo(interrupted -> stop());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Left RPM", () -> leftMotor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty("Middle RPM", () -> middleMotor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty("Right RPM", () -> rightMotor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty("Target RPM", () -> currentRequestedRPM, null);
        builder.addBooleanProperty("At Target Speed", this::isVelocityWithinTolerance, null);
    }
}