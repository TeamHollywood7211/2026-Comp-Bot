// src/main/java/frc/robot/commands/AimAndDriveCommand.java
package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving;
import frc.robot.Landmarks;
import frc.robot.subsystems.Swerve;
import frc.util.DriveInputSmoother;
import frc.util.GeometryUtil;
import frc.util.ManualDriveInput;

public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(2);

    private final Swerve swerve;
    private final DriveInputSmoother inputSmoother;

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withRotationalDeadband(Driving.kPIDRotationDeadband)
            .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withHeadingPID(10, 0, 0.1);

    public AimAndDriveCommand(
            Swerve swerve,
            DoubleSupplier forwardInput,
            DoubleSupplier leftInput) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        addRequirements(swerve);
    }

    public AimAndDriveCommand(Swerve swerve) {
        this(swerve, () -> 0, () -> 0);
    }

    public boolean isAimed() {
        final Rotation2d targetHeading = getDirectionToHub();
        final Rotation2d currentHeading = swerve.getState().Pose.getRotation();
        return GeometryUtil.isNear(targetHeading, currentHeading, kAimTolerance);
    }

    private Rotation2d getDirectionToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        return hubPosition.minus(robotPosition).getAngle();
    }

    @Override
    public void execute() {
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        
        double velocityX = Driving.kMaxSpeed.times(input.forward).in(edu.wpi.first.units.Units.MetersPerSecond);
        double velocityY = Driving.kMaxSpeed.times(input.left).in(edu.wpi.first.units.Units.MetersPerSecond);
        
        Rotation2d forwardDirection = swerve.getOperatorForwardDirection();
        double fieldX = velocityX * forwardDirection.getCos() - velocityY * forwardDirection.getSin();
        double fieldY = velocityX * forwardDirection.getSin() + velocityY * forwardDirection.getCos();

        swerve.setControl(
                fieldCentricFacingAngleRequest
                        .withVelocityX(fieldX)
                        .withVelocityY(fieldY)
                        .withTargetDirection(getDirectionToHub()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}