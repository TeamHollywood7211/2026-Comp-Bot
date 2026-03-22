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
    
    // Remember to use the exact camera names from the PhotonVision web dropdown!
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
        // --- LIMELIGHT CONFIGURATION ---
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

        // --- PHOTONVISION (LUMA) CONFIGURATION ---
        try {
            AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            
            // 1. FRONT LUMA POSITION
            Transform3d robotToLumaFront = new Transform3d(
                new Translation3d(
                    -0.04, // REPLACE: X (Forward) in meters
                    0.0,   // REPLACE: Y (Left) in meters
                    0.6475  // REPLACE: Z (Up) in meters
                ), 
                new Rotation3d(
                    Math.toRadians(0.0),  // Roll
                    Math.toRadians(20.0), // REPLACE: Pitch
                    Math.toRadians(0.0)   // Yaw (0 is forward)
                ) 
            );

            lumaFrontEstimator = new PhotonPoseEstimator(
                layout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                robotToLumaFront
            );

            // 2. REAR USB CAMERA POSITION
            Transform3d robotToLumaRear = new Transform3d(
                new Translation3d(
                    -0.35, // REPLACE: X (Forward) in meters (Negative means back)
                    0.0,    // REPLACE: Y (Left) in meters
                    0.415  // REPLACE: Z (Up) in meters
                ), 
                new Rotation3d(
                    Math.toRadians(0.0),   // Roll
                    Math.toRadians(20.0),  // REPLACE: Pitch
                    Math.toRadians(180.0)  // Yaw (180 degrees faces perfectly backward)
                ) 
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
                applyVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }
        }

        if (lumaFrontEstimator != null) {
            var frontResult = lumaFrontCam.getLatestResult();
            if (frontResult.hasTargets()) {
                Optional<EstimatedRobotPose> frontPose = lumaFrontEstimator.update(frontResult);
                if (frontPose.isPresent()) {
                    applyVisionMeasurement(frontPose.get().estimatedPose.toPose2d(), frontPose.get().timestampSeconds);
                }
            }
        }

        if (lumaRearEstimator != null) {
            var rearResult = lumaRearCam.getLatestResult();
            if (rearResult.hasTargets()) {
                Optional<EstimatedRobotPose> rearPose = lumaRearEstimator.update(rearResult);
                if (rearPose.isPresent()) {
                    applyVisionMeasurement(rearPose.get().estimatedPose.toPose2d(), rearPose.get().timestampSeconds);
                }
            }
        }
    }

    private void applyVisionMeasurement(Pose2d pose, double timestampSeconds) {
        if (!m_hasInitializedVisionPose) {
            this.resetPose(pose);
            m_hasInitializedVisionPose = true;
        } else {
            this.addVisionMeasurement(pose, timestampSeconds, VecBuilder.fill(0.7, 0.7, 999999));
        }
    }
}