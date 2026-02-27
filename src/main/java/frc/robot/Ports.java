package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("main");

    // Talon FX IDs
    public static final int kIntakePivot = 11;
    public static final int kIntakeRollers = 46;
    public static final int kIntakeCanR = 47;
    public static final int kFloor = 45;
    public static final int kFeeder = 48;
    public static final int kShooterLeft = 41;
    public static final int kShooterMiddle = 42;
    public static final int kShooterRight = 43;
    public static final int kHanger = 44;

    // PWM Ports
    public static final int kHoodLeftServo = 0;
    public static final int kHoodRightServo = 1;

    // LimeLight Cam Host Names
    public static final String kLimeLightShooter = "limelight-right";
}
