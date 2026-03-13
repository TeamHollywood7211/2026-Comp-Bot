package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Landmarks;
import frc.robot.LimelightHelpers;
import frc.robot.Ports;
import frc.robot.generated.TunerConstants;

public class Swerve extends CommandSwerveDrivetrain {
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity);

    public Swerve() {
        super(
                TunerConstants.DrivetrainConstants,
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight);

        configurePathPlanner();
    }

    private void configurePathPlanner() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    () -> this.getState().Pose,
                    this::resetPose,
                    () -> this.getState().Speeds,
                    (speeds, feedforwards) -> setControl(
                            autoRequest.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            new PIDConstants(5.0, 0.0, 0.0),
                            new PIDConstants(7.0, 0.0, 0.0)),
                    config,
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this);
        } catch (Exception e) {
            DriverStation.reportError("PathPlanner config failed: " + e.getMessage(), e.getStackTrace());
        }
    }

    public Pose2d getStatePose() {
        return this.getState().Pose;
    }

    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public double getDistanceToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = this.getState().Pose.getTranslation();
        return robotPosition.getDistance(hubPosition);
    }

@Override
    public void periodic() {
        // Remove the DriverStation.isDisabled() check so it locks in the perspective once
        if (!m_hasAppliedOperatorPerspective) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                
                if (!DriverStation.isAutonomous()) {
                    seedFieldCentric();
                }
                m_hasAppliedOperatorPerspective = true;
            });
        }
        
        updateVision();
        SmartDashboard.putNumber("Shooter/Distance To Hub", getDistanceToHub());
    }

    private void updateVision() {
        Pose2d currentPose = this.getState().Pose;
        double yaw = currentPose.getRotation().getDegrees();
        
        LimelightHelpers.SetRobotOrientation(Ports.kLimeLightShooter, yaw, 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(Ports.kLimeLightShooter);

        if (mt2 != null && mt2.tagCount > 0) {
            double distanceError = mt2.pose.getTranslation().getDistance(currentPose.getTranslation());
            
            if (distanceError < 1.0) {
                double xyStdDev = mt2.tagCount > 1 ? 0.3 : 0.7;
                this.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, 999999));
                this.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }
        }
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }
}