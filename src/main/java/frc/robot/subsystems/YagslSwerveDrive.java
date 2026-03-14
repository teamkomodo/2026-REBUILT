package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FFGains;
import frc.robot.util.PIDGains;
import frc.robot.util.Util;
import frc.robot.util.YagslModule;

import static frc.robot.Constants.*;

import java.util.function.DoubleSupplier;

// Example SwerveDrive class
public class YagslSwerveDrive extends SubsystemBase {
    
    // Attributes
    private final SwerveDrivePoseEstimator odometry;
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k200Hz);

    private final YagslModule frontLeft;
    private final YagslModule frontRight;
    private final YagslModule backLeft;
    private final YagslModule backRight;

    private final Translation2d frontLeftPosition = new Translation2d(DRIVETRAIN_LENGTH / 2D,  DRIVETRAIN_WIDTH / 2D);
    private final Translation2d frontRightPosition = new Translation2d(DRIVETRAIN_LENGTH / 2D, -DRIVETRAIN_WIDTH / 2D);
    private final Translation2d backLeftPosition  = new Translation2d(-DRIVETRAIN_LENGTH / 2D,  DRIVETRAIN_WIDTH / 2D);
    private final Translation2d backRightPosition = new Translation2d(-DRIVETRAIN_LENGTH / 2D, -DRIVETRAIN_WIDTH / 2D);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftPosition, 
        frontRightPosition,
        backLeftPosition, 
        backRightPosition);
    
    private boolean slowMode = false;
    private double brakeModeScale = 1.9;

    public static final NetworkTable drivetrainNT = NetworkTableInstance.getDefault().getTable("drivetrain");

    private final StructArrayPublisher<SwerveModuleState> measuredSwerveStatesPublisher = drivetrainNT
        .getStructArrayTopic("measuredSwerveStates", SwerveModuleState.struct).publish();

    private final StructArrayPublisher<SwerveModuleState> desiredSwerveStatesPublisher = drivetrainNT
        .getStructArrayTopic("desiredSwerveStates", SwerveModuleState.struct).publish();

    private final StructPublisher<Pose2d> robotPosePublisher = drivetrainNT
        .getStructTopic("robotPose", Pose2d.struct).publish();

    private final StructPublisher<Rotation2d> adjustedRotationPublisher = drivetrainNT
        .getStructTopic("adjustedRotation", Rotation2d.struct).publish();

    private final StructPublisher<Rotation2d> rotationPublisher = drivetrainNT
        .getStructTopic("rotation", Rotation2d.struct).publish();
    
    // Constructor
    public YagslSwerveDrive() {

        frontLeft = new YagslModule(
                FRONT_LEFT_DRIVE_MOTOR_ID,
                FRONT_LEFT_STEER_MOTOR_ID,
                FRONT_LEFT_STEER_ENCODER_ID,
                FRONT_LEFT_STEER_OFFSET,
                new PIDGains(1.0, 0, 0),
                new PIDGains(1, 1.0e-6, 0),
                new FFGains(0.19861, 3.2379, 0.562),
                drivetrainNT.getSubTable("frontLeft")
                // new FFGains(1, 0, 0),
                );

        frontRight = new YagslModule(
                FRONT_RIGHT_DRIVE_MOTOR_ID,
                FRONT_RIGHT_STEER_MOTOR_ID,
                FRONT_RIGHT_STEER_ENCODER_ID,
                FRONT_RIGHT_STEER_OFFSET,
                new PIDGains(1.0, 0, 0),
                new PIDGains(1, 1.0e-6, 0),
                new FFGains(0.18406, 3.2722, 0.40914),
                drivetrainNT.getSubTable("frontRight")
                // new FFGains(1, 0, 0),
                );

        backLeft = new YagslModule(
                BACK_LEFT_DRIVE_MOTOR_ID,
                BACK_LEFT_STEER_MOTOR_ID,
                BACK_LEFT_STEER_ENCODER_ID,
                BACK_LEFT_STEER_OFFSET,
                new PIDGains(1.0, 0, 0),
                new PIDGains(1, 1.0e-6, 0),
                new FFGains(0.17395, 3.286, 0.51328),
                drivetrainNT.getSubTable("backLeft")
                // new FFGains(1, 0, 0),
                );

        backRight = new YagslModule(
                BACK_RIGHT_DRIVE_MOTOR_ID,
                BACK_RIGHT_STEER_MOTOR_ID,
                BACK_RIGHT_STEER_ENCODER_ID,
                BACK_RIGHT_STEER_OFFSET,
                new PIDGains(1.0, 0, 0),
                new PIDGains(1, 1.0e-6, 0),
                new FFGains(0.17731, 3.2446, 0.41604),
                drivetrainNT.getSubTable("backRight")
                // new FFGains(1, 0, 0),
                );
        

        // Create the SwerveDriveOdometry given the current angle, the robot is at x=0, r=0, and heading=0
        odometry = new SwerveDrivePoseEstimator(
            kinematics,
            getRotation(), // returns current gyro reading as a Rotation2d
            new SwerveModulePosition[] {
                frontRight.getPosition(),
                frontLeft.getPosition(),
                backRight.getPosition(),
                backLeft.getPosition()},
            // Front-Left, Front-Right, Back-Left, Back-Right
            new Pose2d(10, 0, new Rotation2d(-90)) // x=10, y=0, heading=-90
        );
            
    }
    
    // Simple drive function
    public void drive(double xSpeed, double ySpeed, double angularVelocity, boolean fieldRelative) {
        // Create test ChassisSpeeds going X = 14in, Y=4in, and spins at 30deg per second.
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, angularVelocity);
        // Get the SwerveModuleStates for each module given the desired speeds.
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_MODULE_VELOCITY);
        // Output order is Front-Left, Front-Right, Back-Left, Back-Right
        setModuleStates(swerveModuleStates);
    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, fieldRelative);
    }
    
    // Fetch the current swerve module positions.
    public SwerveModulePosition[] getCurrentSwerveModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        frontLeft.setDesiredState(moduleStates[0]);// 0
        frontRight.setDesiredState(moduleStates[1]);// 1
        backLeft.setDesiredState(moduleStates[2]);// 2
        backRight.setDesiredState(moduleStates[3]);// 3
    }

    public ChassisSpeeds joystickAxesToChassisSpeeds(double xAxis, double yAxis, double rotAxis) {

        double xVelocity = Util.translationCurve(MathUtil.applyDeadband(xAxis, XBOX_DEADBAND))
                * LINEAR_VELOCITY_CONSTRAINT * (slowMode ? LINEAR_SLOW_MODE_MODIFIER : 1);
        double yVelocity = Util.translationCurve(MathUtil.applyDeadband(yAxis, XBOX_DEADBAND))
                * LINEAR_VELOCITY_CONSTRAINT * (slowMode ? LINEAR_SLOW_MODE_MODIFIER : 1);
        double rotVelocity = Util.steerCurve(MathUtil.applyDeadband(rotAxis, XBOX_DEADBAND))
                * ANGULAR_VELOCITY_CONSTRAINT * (slowMode ? ANGULAR_SLOW_MODE_MODIFIER : 1);

        double totalVelocity = Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(yVelocity, 2));

        if (totalVelocity > LINEAR_VELOCITY_CONSTRAINT) {
            xVelocity *= (LINEAR_VELOCITY_CONSTRAINT / totalVelocity);
            yVelocity *= (LINEAR_VELOCITY_CONSTRAINT / totalVelocity);
        }

        return new ChassisSpeeds(xVelocity, yVelocity, rotVelocity);
    }

    public Command joystickDriveCommand(DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rotAxis) {
        return Commands.run(() -> {
            double oX = xAxis.getAsDouble();
            double oY = yAxis.getAsDouble();
            double oR = rotAxis.getAsDouble();

            // Leave this here!
            double x = oX * brakeModeScale;
            double y = oY * brakeModeScale;
            double r = oR * Math.pow(brakeModeScale, 0.7);

            ChassisSpeeds speeds = joystickAxesToChassisSpeeds(oX, oY, oR);
            drive(speeds, true);
        }, this);
    }
    
    @Override
    public void periodic() {
        // Update the odometry every run.
        odometry.update(getRotation(), getCurrentSwerveModulePositions());
        updateTelemetry();
    }

    public Rotation2d getRotation() {
        return gyro.getRotation2d().plus(Rotation2d.fromRadians(Math.PI));
    }

    public Rotation2d getAdjustedRotation() {
        return getRotation().plus(Rotation2d.fromRadians(0 + Math.PI));
    }

    private void updateTelemetry() {
        // Swerve
        desiredSwerveStatesPublisher.set(new SwerveModuleState[] {
                frontLeft.getDesiredState(),
                frontRight.getDesiredState(),
                backLeft.getDesiredState(),
                backRight.getDesiredState()
        });

        measuredSwerveStatesPublisher.set(new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        }, RobotController.getFPGATime() - 200000);

        adjustedRotationPublisher.set(getAdjustedRotation());
        rotationPublisher.set(getRotation());

        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();

        //robotPosePublisher.set(getPose()); FIXME
    }
    
}