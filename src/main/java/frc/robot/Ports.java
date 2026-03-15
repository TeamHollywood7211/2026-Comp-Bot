// src/main/java/frc/robot/Ports.java
package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("main");

    public static final int kIntakePivot = 11;
    public static final int kIntakeRollers = 46;
    public static final int kInatkeRollers2 = 52;
    public static final int kIntakeCanR = 47;
    public static final int kFloor = 48;
    public static final int kFeeder = 45;
    public static final int kShooterLeft = 41;
    public static final int kShooterMiddle = 42;
    public static final int kShooterRight = 43;
    public static final int kHanger = 44;
    public static final int kCANdle = 50;
    public static final int kFrontRange = 51;


    public static final int kHoodLeftServo = 0;
    public static final int kHoodRightServo = 1;
    public static final int kIntakeLimitSwitch = 2;

    public static final String kLimeLightShooter = "limelight-right";
}