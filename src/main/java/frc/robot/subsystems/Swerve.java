package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
    private boolean m_hasInitializedVisionPose = false;

    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity);

    private final Field2d field = new Field2d();

    public Swerve() {
        super(
                TunerConstants.DrivetrainConstants,
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight);

        SmartDashboard.putData("Field", field);
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
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
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
        
        field.setRobotPose(this.getState().Pose);
        updateVision();
        SmartDashboard.putNumber("Shooter/Distance To Hub", getDistanceToHub());
    }

    private void updateVision() {
        double yaw = this.getState().Pose.getRotation().getDegrees();
        double yawRate = Math.toDegrees(this.getState().Speeds.omegaRadiansPerSecond);
        LimelightHelpers.SetRobotOrientation(Ports.kLimeLightShooter, yaw, yawRate, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(Ports.kLimeLightShooter);

        if (mt2 != null) {
            SmartDashboard.putNumber("Vision/Tag Count", mt2.tagCount);
            
            if (mt2.tagCount > 0) {
                SmartDashboard.putNumber("Vision/Raw X", mt2.pose.getX());
                SmartDashboard.putNumber("Vision/Raw Y", mt2.pose.getY());

                if (!m_hasInitializedVisionPose) {
                    this.resetPose(mt2.pose);
                    m_hasInitializedVisionPose = true;
                } else {
                    this.addVisionMeasurement(
                        mt2.pose, 
                        mt2.timestampSeconds, 
                        VecBuilder.fill(0.7, 0.7, 999999)
                    );
                }
            }
        }
    }
}