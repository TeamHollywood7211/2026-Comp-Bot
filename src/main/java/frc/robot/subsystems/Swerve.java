package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.robot.Landmarks;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

public class Swerve extends CommandSwerveDrivetrain {
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;
    private boolean m_hasInitializedVisionPose = false;

    private final String[] limelightNames = { "limelight-left", "limelight-right" };
    
    private final PhotonCamera lumaFrontCam = new PhotonCamera("luma-front"); 
    private PhotonPoseEstimator lumaFrontEstimator;

    private final PhotonCamera lumaRearCam = new PhotonCamera("luma-back");
    private PhotonPoseEstimator lumaRearEstimator;

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
        configureVision();
        configurePathPlanner();
    }

    private void configureVision() {
        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-left", 
            -0.0325,    
            -0.3,  
            0.0975,  
            0.0,    
            20.0,    
            90    
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-right", 
            -0.0375,     
            0.3,  
            0.0975,   
            0.0,     
            20.0,     
            -90    
        );

        try {
            AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            
            Transform3d robotToLumaFront = new Transform3d(
                new Translation3d(-0.04, 0.0, 0.6475), 
                new Rotation3d(Math.toRadians(0.0), Math.toRadians(20.0), Math.toRadians(0.0)) 
            );

            lumaFrontEstimator = new PhotonPoseEstimator(
                layout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                robotToLumaFront
            );

            Transform3d robotToLumaRear = new Transform3d(
                new Translation3d(-0.35, 0.0, 0.415), 
                new Rotation3d(Math.toRadians(0.0), Math.toRadians(20.0), Math.toRadians(180.0)) 
            );

            lumaRearEstimator = new PhotonPoseEstimator(
                layout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                robotToLumaRear
            );

        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTag layout: " + e.getMessage(), false);
        }
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

        for (String camName : limelightNames) {
            LimelightHelpers.SetRobotOrientation(camName, yaw, yawRate, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camName);

            if (mt2 != null && mt2.tagCount > 0) {
                // Limelight MT2 relies heavily on gyro. Distance scaling helps prevent jumpiness.
                double distance = mt2.avgTagDist;
                double stdDev = mt2.tagCount > 1 ? 0.4 : 0.8 + (distance * 0.2);
                applyVisionMeasurement(mt2.pose, mt2.timestampSeconds, stdDev);
            }
        }

        if (lumaFrontEstimator != null) {
            var frontResult = lumaFrontCam.getLatestResult();
            if (frontResult.hasTargets()) {
                boolean rejectPose = frontResult.getTargets().size() == 1 && frontResult.getBestTarget().getPoseAmbiguity() > 0.2;
                if (!rejectPose) {
                    Optional<EstimatedRobotPose> frontPose = lumaFrontEstimator.update(frontResult);
                    if (frontPose.isPresent()) {
                        double distance = frontResult.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
                        double stdDev = frontResult.getTargets().size() > 1 ? 0.3 : 0.7 + (distance * 0.2);
                        applyVisionMeasurement(frontPose.get().estimatedPose.toPose2d(), frontPose.get().timestampSeconds, stdDev);
                    }
                }
            }
        }

        if (lumaRearEstimator != null) {
            var rearResult = lumaRearCam.getLatestResult();
            if (rearResult.hasTargets()) {
                boolean rejectPose = rearResult.getTargets().size() == 1 && rearResult.getBestTarget().getPoseAmbiguity() > 0.2;
                if (!rejectPose) {
                    Optional<EstimatedRobotPose> rearPose = lumaRearEstimator.update(rearResult);
                    if (rearPose.isPresent()) {
                        double distance = rearResult.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
                        double stdDev = rearResult.getTargets().size() > 1 ? 0.3 : 0.7 + (distance * 0.2);
                        applyVisionMeasurement(rearPose.get().estimatedPose.toPose2d(), rearPose.get().timestampSeconds, stdDev);
                    }
                }
            }
        }
    }

    private void applyVisionMeasurement(Pose2d pose, double timestampSeconds, double stdDev) {
        if (!m_hasInitializedVisionPose) {
            this.resetPose(pose);
            m_hasInitializedVisionPose = true;
        } else {
            // Cap standard deviation at an extremely untrustworthy number if it spikes
            stdDev = Math.min(stdDev, 5.0);
            this.addVisionMeasurement(pose, timestampSeconds, VecBuilder.fill(stdDev, stdDev, 999999));
        }
    }
}