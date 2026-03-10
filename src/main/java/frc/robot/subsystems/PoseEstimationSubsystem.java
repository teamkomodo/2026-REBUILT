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
                                                                          // addtl cameras
    // later
    private final PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
    private final DrivetrainSubsystem drivetrainSubsystem;

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
            drivetrainSubsystem.addVisionMeasurement(
                    est.estimatedPose.toPose2d(),
                    est.timestampSeconds);
        });
    }

    /**
     * Returns a Rotation2d to the team hub
     * 
     * @return
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
            /*
             * Pose3d pose = getVisionPose();
             * System.out.print("Pose: ");
             * System.out.print(pose.getX());
             * System.out.print(", ");
             * System.out.print(pose.getY());
             * System.out.print(", ");
             * System.out.println(pose.getZ());
             */
        });
    }

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