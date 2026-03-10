package frc.robot.subsystems;

import static frc.robot.Constants.*;

import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.photonvision.*;

public class PoseEstimationSubsystem extends SubsystemBase {
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0, 0)); // Tune me
    private final PhotonCamera camera = new PhotonCamera("photonvision"); // Todo: configure as front cam, allow for
                                                                          // addtl cameras later
    private final PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
    private final DrivetrainSubsystem drivetrainSubsystem;

    private Optional<EstimatedRobotPose> lastVisionPose;

    public PoseEstimationSubsystem(DrivetrainSubsystem drivetrain) {
        this.drivetrainSubsystem = drivetrain;
    }

    public Optional<EstimatedRobotPose> getVisionPose() {

        for (var result : camera.getAllUnreadResults()) {

            var est = photonEstimator.estimateCoprocMultiTagPose(result);

            if (est.isEmpty()) {
                est = photonEstimator.estimateLowestAmbiguityPose(result);
            }

            if (est.isPresent()) {
                lastVisionPose = est;
                return est;
            }
        }
        // Use below for insane estimation accuracy
        // visionEst.ifPresent(
        // est -> {
        // // Change our trust in the measurement based on the tags we can see
        // var estStdDevs = getEstimationStdDevs();

        // estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds,
        // estStdDevs);
        // }); // Use below for insane estimation accuracy
        // visionEst.ifPresent(
        // est -> {
        // // Change our trust in the measurement based on the tags we can see
        // var estStdDevs = getEstimationStdDevs();

        // estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds,
        // estStdDevs);
        // });

        return Optional.empty();
    }

    @Override
    public void periodic() {
        var pose = getVisionPose();
        pose.ifPresent(est -> {
            if (true) { // Check whether current vision position - lastVisionPosition is less than max
                        // velocity * delta time to avoid teleportation
                drivetrainSubsystem.addVisionMeasurement(
                        est.estimatedPose.toPose2d(),
                        est.timestampSeconds);
            }
        });
    }

    /**
     * Returns a Rotation2d to the team hub
     * 
     * @return Rotation2d representing angle between robot and hub (use
     *         .getRadians() / .getDegrees() for a value)
     */
    public Rotation2d getRotationToHub() {
        Translation2d hubPosMeters;
        if (ON_RED_ALLIANCE.getAsBoolean()) { // FIXME: Replace placeholders with actual hub positions
            hubPosMeters = new Translation2d(0, 0); // Red hub position
        } else { // Blue alliance
            hubPosMeters = new Translation2d(1, 1); // Blue hub position
        }
        return hubPosMeters
                .minus(drivetrainSubsystem.getPoseEstimation().getTranslation())
                .getAngle();
    }

    public Command printVisionPoseEstimation() {
        return Commands.runOnce(() -> {
            lastVisionPose.ifPresent(est -> {
                System.out.print("========Timestamp: ");
                System.out.println(est.timestampSeconds);
                System.out.print("========Pose: X: ");
                System.out.print(est.estimatedPose.getX());
                System.out.print(", Y: ");
                System.out.println(est.estimatedPose.getY());
                System.out.print("========Rotation: Angle: ");
                System.out.print(est.estimatedPose.getRotation().getZ()); // Verify if z is right
                System.out.print(" degrees.");
            });

        });
    }

    public Command printDrivetrainPoseEstimation() {
        Pose2d drivetrainPoseEstimation = drivetrainSubsystem.getPoseEstimation();
        return Commands.runOnce(() -> {
            System.out.print("========Pose: X: ");
            System.out.print(drivetrainPoseEstimation.getX());
            System.out.print(", Y: ");
            System.out.print(drivetrainPoseEstimation.getY());
            System.out.print("========Rotation: Angle: ");
            System.out.print(drivetrainPoseEstimation.getRotation().getDegrees()); // Verify if z is right
            System.out.print(" degrees.");
        });

    };

    public double getDistanceToHubCenterMeters() {
        Translation2d hubPosMeters;
        if (ON_RED_ALLIANCE.getAsBoolean()) { // FIXME: Replace placeholders with actual hub positions
            hubPosMeters = new Translation2d(0, 0); // Red hub position
        } else {
            hubPosMeters = new Translation2d(1, 1); // Blue hub position
        }
        return drivetrainSubsystem.getPoseEstimation().getTranslation().getDistance(hubPosMeters);
    }
}