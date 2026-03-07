package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
    private final DoubleConsumer setShooterRpm;
    
    private final InterpolatingDoubleTreeMap distanceToRpmMap = new InterpolatingDoubleTreeMap();

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withRotationalDeadband(Driving.kPIDRotationDeadband)
            .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withHeadingPID(10, 0, 0.1);

    public AimAndDriveCommand(
            Swerve swerve,
            DoubleSupplier forwardInput,
            DoubleSupplier leftInput,
            DoubleConsumer setShooterRpm) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        this.setShooterRpm = setShooterRpm;
        addRequirements(swerve);

        distanceToRpmMap.put(1.5, 3000.0);
        distanceToRpmMap.put(3.0, 4500.0);
        distanceToRpmMap.put(5.0, 5500.0);
    }

    public AimAndDriveCommand(
            Swerve swerve,
            DoubleSupplier forwardInput,
            DoubleSupplier leftInput) {
        this(swerve, forwardInput, leftInput, null);
    }

    public AimAndDriveCommand(Swerve swerve) {
        this(swerve, () -> 0, () -> 0, null);
    }

    public boolean isAimed() {
        final Rotation2d targetHeading = fieldCentricFacingAngleRequest.TargetDirection;
        final Rotation2d currentHeadingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
        
        final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective
                .minus(swerve.getOperatorForwardDirection());
        
        return GeometryUtil.isNear(targetHeading, currentHeadingInOperatorPerspective, kAimTolerance);
    }

    private Rotation2d getDirectionToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();

        Rotation2d fieldRelativeAngle = hubPosition.minus(robotPosition).getAngle();
        return fieldRelativeAngle.minus(swerve.getOperatorForwardDirection());
    }

    private double getDistanceToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        return robotPosition.getDistance(hubPosition);
    }

    @Override
    public void execute() {
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        swerve.setControl(
                fieldCentricFacingAngleRequest
                        .withVelocityX(Driving.kMaxSpeed.times(input.forward))
                        .withVelocityY(Driving.kMaxSpeed.times(input.left))
                        .withTargetDirection(getDirectionToHub()));

        if (setShooterRpm != null) {
            double distanceMeters = getDistanceToHub();
            setShooterRpm.accept(distanceToRpmMap.get(distanceMeters));
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}