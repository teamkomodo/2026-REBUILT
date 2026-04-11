package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.photonvision.*;

public class PoseEstimationSubsystem extends SubsystemBase {
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltWelded);
    public static final Transform3d kRobotToCam = new Transform3d(
            new Translation3d(-11 * 2.54 / 100, -6 * 2.54 / 100, -6 * 2.54 / 100),
            new Rotation3d(0, 17.0 * Math.PI / 180, 0)); // Tune me
    private final PhotonCamera camera = new PhotonCamera("photonvision"); // Todo: configure as front cam, allow for
                                                                   // addtl cameras later
    private final PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
    private final DrivetrainSubsystem drivetrainSubsystem;

    private Optional<EstimatedRobotPose> lastVisionPose = Optional.empty();

    public PoseEstimationSubsystem(DrivetrainSubsystem drivetrain) {
        this.drivetrainSubsystem = drivetrain;
    }

    public Optional<EstimatedRobotPose> getVisionPose() {
        for (var result : camera.getAllUnreadResults()) {
            if (!result.hasTargets()) {
                continue;
            }
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

    }

    public void visionTeleopPeriodic() {
        var pose = getVisionPose();
        pose.ifPresent(est -> {
            Pose2d incomingVisionPose = est.estimatedPose.toPose2d();
            Pose2d currentPose = drivetrainSubsystem.getPoseEstimation();

            double error = incomingVisionPose.getTranslation().getDistance(currentPose.getTranslation());

            if (error < 2 || est.targetsUsed.size() >= 2) { // I completely pulled this error rate out of my butt, we
                                                            // need to some
                                                            // velocity adjustment FIXME: Change 1 to 2 on real
                                                            // field!!!!
                // for this
                drivetrainSubsystem.addVisionMeasurement(
                        incomingVisionPose,
                        est.timestampSeconds, VecBuilder.fill(0.5, 0.5, 60 * Math.PI / 180)); // TODO: Add
                                                                                              // VisionStdDevs
                                                                                              // to improve pose
                                                                                              // estimation by a
                                                                                              // lot, fix placeholder
            }
        });
    }

    public void visionAutoPeriodic() {
        var pose = getVisionPose();
        pose.ifPresent(est -> {
            Pose2d incomingVisionPose = est.estimatedPose.toPose2d();
            Pose2d currentPose = drivetrainSubsystem.getPoseEstimation();

            double error = incomingVisionPose.getTranslation().getDistance(currentPose.getTranslation());

            if (error < 2 || est.targetsUsed.size() >= 2) { // I completely pulled this error rate out of my butt, we
                                                            // need to some
                                                            // velocity adjustment FIXME: Change 1 to 2 on real
                                                            // field!!!!
                // for this
                drivetrainSubsystem.addVisionMeasurement(
                        incomingVisionPose.rotateBy(Rotation2d.fromRadians(Math.PI)),
                        est.timestampSeconds, VecBuilder.fill(0.5, 0.5, 60 * Math.PI / 180)); // TODO: Add
                                                                                              // VisionStdDevs
                                                                                              // to improve pose
                                                                                              // estimation by a
                                                                                              // lot, fix placeholder
            }
        });
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
                System.out.print(est.estimatedPose.getRotation().getZ() * 180 / Math.PI); // Verify if z is right
                System.out.print(" degrees.");
            });

        });
    }

    public Command printDrivetrainPoseEstimation() {
        return Commands.runOnce(() -> {
            Pose2d drivetrainPoseEstimation = drivetrainSubsystem.getPoseEstimation();
            System.out.print("========Pose: X: ");
            System.out.print(drivetrainPoseEstimation.getX());
            System.out.print(", Y: ");
            System.out.print(drivetrainPoseEstimation.getY());
            System.out.print("========Rotation: Angle: ");
            System.out.print(drivetrainPoseEstimation.getRotation().getDegrees()); // Verify if z is right
            System.out.println(" degrees.");
            
        });

    };

    // PoseEstimation wrapper func for drivetrain function so shooter can access it
    public double getDistanceToHubCenterMeters() {
        return drivetrainSubsystem.getDistanceToHubCenterMeters();
    }
}