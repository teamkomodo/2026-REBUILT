package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.FFGains;
import frc.robot.util.NeoSwerveModule;
import frc.robot.util.PIDGains;
import frc.robot.util.SwerveModule;
import frc.robot.util.Util;

import static frc.robot.Constants.*;

import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DrivetrainSubsystem implements Subsystem {

    // Limelight
    private static boolean useVision = false;

    private final NetworkTable limelightNT = NetworkTableInstance.getDefault().getTable("limelight-komodo");
    private final DoubleSubscriber validTargetSubscriber = limelightNT.getDoubleTopic("tv").subscribe(0);
    private final DoubleArraySubscriber botPoseBlueSubscriber = limelightNT.getDoubleArrayTopic("botpose_wpiblue")
            .subscribe(new double[0]);

    public boolean speedMode = !false;
    private double brakeModeScale = 0;
    public boolean atReef = false;

    // Telemetry
    public static final NetworkTable drivetrainNT = NetworkTableInstance.getDefault().getTable("drivetrain");

    public final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT);

    private final StructArrayPublisher<SwerveModuleState> measuredSwerveStatesPublisher = drivetrainNT
            .getStructArrayTopic(
                    "measuredSwerveStates",
                    SwerveModuleState.struct)
            .publish();

    private final StructArrayPublisher<SwerveModuleState> desiredSwerveStatesPublisher = drivetrainNT
            .getStructArrayTopic(
                    "desiredSwerveStates",
                    SwerveModuleState.struct)
            .publish();

    private final StructPublisher<Pose2d> robotPosePublisher = drivetrainNT.getStructTopic("robotPose", Pose2d.struct)
            .publish();

    private final StructPublisher<Rotation2d> adjustedRotationPublisher = drivetrainNT.getStructTopic(
            "adjustedRotation",
            Rotation2d.struct).publish();

    private final StructPublisher<Rotation2d> rotationPublisher = drivetrainNT.getStructTopic(
            "rotation",
            Rotation2d.struct).publish();

    // SysID
    private final SysIdRoutine driveSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    (voltage) -> runDriveVolts(voltage.in(Units.Volts)),
                    null,
                    this));

    private final SysIdRoutine steerSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    (voltage) -> runSteerVolts(voltage.in(Units.Volts)),
                    null,
                    this));

    // Swerve
    // WPILib: x = forward/back (length), y = left/right (width)
    private final Translation2d frontLeftPosition = new Translation2d(DRIVETRAIN_LENGTH / 2D,  DRIVETRAIN_WIDTH / 2D);
    private final Translation2d frontRightPosition = new Translation2d(DRIVETRAIN_LENGTH / 2D, -DRIVETRAIN_WIDTH / 2D);
    private final Translation2d backLeftPosition  = new Translation2d(-DRIVETRAIN_LENGTH / 2D,  DRIVETRAIN_WIDTH / 2D);
    private final Translation2d backRightPosition = new Translation2d(-DRIVETRAIN_LENGTH / 2D, -DRIVETRAIN_WIDTH / 2D);

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftPosition, frontRightPosition,
            backLeftPosition, backRightPosition);
    private final SwerveDrivePoseEstimator poseEstimator;
    private final ProfiledPIDController rotationController = new ProfiledPIDController(3, 1.0e-5, 1.0e-1,
            new TrapezoidProfile.Constraints(ANGULAR_VELOCITY_CONSTRAINT, ANGULAR_ACCEL_CONSTRAINT));
    private final HolonomicDriveController driveController = new HolonomicDriveController(
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            rotationController);

    private final AHRS navX = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k200Hz);// new AHRS(SPI.Port.kMXP,
                                                                                              // (byte) 200);
    private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();
    RobotConfig config;
    private boolean slowMode = false;
    private double rotationOffsetRadians = 0;

    private ChassisSpeeds lastCommandedChassisSpeeds = new ChassisSpeeds();

    public DrivetrainSubsystem() {

        // only tracks specific apriltags depending on alliance
        // LimelightHelpers.SetFiducialIDFiltersOverride("limelight-komodo", new
        // int[]{11,10,9,8,7,6, 22,21,20,19,18,17});
        // Drive FFGain updated AM 03/07
        frontLeft = new NeoSwerveModule(
                FRONT_LEFT_DRIVE_MOTOR_ID,
                FRONT_LEFT_STEER_MOTOR_ID,
                FRONT_LEFT_STEER_ENCODER_ID,
                FRONT_LEFT_STEER_OFFSET,
                new PIDGains(1.0, 0, 0),
                new PIDGains(1, 1.0e-6, 0),
                new FFGains(0.19861, 3.2379, 0.562),
                // new FFGains(1, 0, 0),
                drivetrainNT.getSubTable("frontleft"));

        frontRight = new NeoSwerveModule(
                FRONT_RIGHT_DRIVE_MOTOR_ID,
                FRONT_RIGHT_STEER_MOTOR_ID,
                FRONT_RIGHT_STEER_ENCODER_ID,
                FRONT_RIGHT_STEER_OFFSET,
                new PIDGains(1.0, 0, 0),
                new PIDGains(1, 1.0e-6, 0),
                new FFGains(0.18406, 3.2722, 0.40914),
                // new FFGains(1, 0, 0),
                drivetrainNT.getSubTable("frontright"));

        backLeft = new NeoSwerveModule(
                BACK_LEFT_DRIVE_MOTOR_ID,
                BACK_LEFT_STEER_MOTOR_ID,
                BACK_LEFT_STEER_ENCODER_ID,
                BACK_LEFT_STEER_OFFSET,
                new PIDGains(1.0, 0, 0),
                new PIDGains(1, 1.0e-6, 0),
                new FFGains(0.17395, 3.286, 0.51328),
                // new FFGains(1, 0, 0),
                drivetrainNT.getSubTable("backleft"));

        backRight = new NeoSwerveModule(
                BACK_RIGHT_DRIVE_MOTOR_ID,
                BACK_RIGHT_STEER_MOTOR_ID,
                BACK_RIGHT_STEER_ENCODER_ID,
                BACK_RIGHT_STEER_OFFSET,
                new PIDGains(1.0, 0, 0),
                new PIDGains(1, 1.0e-6, 0),
                new FFGains(0.17731, 3.2446, 0.41604),
                // new FFGains(1, 0, 0),
                drivetrainNT.getSubTable("backright"));

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getRotation(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                new Pose2d());
        resetPose(new Pose2d(new Translation2d(10, 0), Rotation2d.fromDegrees(-90)));
        zeroGyro();
        setupPathPlanner();
    }

    void resetAutoPose(Pose2d pose) {
        poseEstimator.resetPosition(getRotation(),
                getSwervePositions(),
                new Pose2d(new Translation2d(10, 0),
                        Rotation2d.fromDegrees(179.79)));
    }

    double lastOffset = 0;

    @Override
    public void periodic() {
        // does not need to use adjusted rotation, odometry handles it.
        // updates pose with rotation and swerve positions
        poseEstimator.update(getRotation(), getSwervePositions());

        updateTelemetry();

        transferBrakeMode();

        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();
    }

    public void robotRelativeDrive(ChassisSpeeds chassisSpeeds, DriveFeedforwards driveFeedforwards) {
        ChassisSpeeds realAutoChassisSpeeds = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond, -chassisSpeeds.omegaRadiansPerSecond);
        drive(realAutoChassisSpeeds, false);
    }

    private void setupPathPlanner() {
        // try {
        // config = RobotConfig.fromGUISettings();
        // AutoBuilder.configure(
        // this::getPose,
        // this::resetPose,
        // this::getChassisSpeeds,
        // this::robotRelativeDrive,
        // HOLONOMIC_PATH_FOLLOWER_CONFIG,
        // config,

        // this
        // );
        // } catch (Exception e){
        // e.printStackTrace();
        // }
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

        frontLeft.updateTelemetry();
        frontRight.updateTelemetry();
        backLeft.updateTelemetry();
        backRight.updateTelemetry();

        robotPosePublisher.set(getPose());
    }

    // tracks position with vision
    private void visionPosePeriodic() {

        // Return if the limelight doesn't see a target
        if (validTargetSubscriber.get() != 1)
            return;

        double[] botPose = botPoseBlueSubscriber.get();
        if (botPose.length < 7)
            return;

        // Convert double[] from NT to Pose2D
        Pose2d visionPose = new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5]));
        double measurementTime = Timer.getFPGATimestamp() - botPose[6] / 1000; // calculate the actual time the picture
                                                                               // was taken

        poseEstimator.addVisionMeasurement(visionPose, measurementTime);
    }

    public void drive(double xSpeed, double ySpeed, double angularVelocity, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, angularVelocity);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                chassisSpeeds, getAdjustedRotation())
                        : chassisSpeeds);

        // var moduleStates = kinematics.toSwerveModuleStates(
        // ChassisSpeeds.discretize(
        // fieldRelative
        // ? ChassisSpeeds.fromFieldRelativeSpeeds(
        // xSpeed, ySpeed, angularVelocity, getAdjustedRotation())
        // : new ChassisSpeeds(xSpeed, ySpeed, angularVelocity), periodSeconds));

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_MODULE_VELOCITY);
        setModuleStates(moduleStates);
    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, fieldRelative);
    }

    private long lastTime = 0;

    private void desaturateChassisSpeedsAcceleration(ChassisSpeeds speeds) {
        long currentTime = RobotController.getFPGATime();
        double dtSeconds = (currentTime - lastTime) / 1e6;
        lastTime = currentTime;

        double accelX = (speeds.vxMetersPerSecond - lastCommandedChassisSpeeds.vxMetersPerSecond) / dtSeconds;
        double accelY = (speeds.vyMetersPerSecond - lastCommandedChassisSpeeds.vyMetersPerSecond) / dtSeconds;
        double linearAccelMag = Math.sqrt(accelX * accelX + accelY * accelY);

        // No need to limit decceleration ie. return if acceleration is in the opposite
        // direction as current travel
        if (accelX * lastCommandedChassisSpeeds.vxMetersPerSecond < 0
                && accelY * lastCommandedChassisSpeeds.vyMetersPerSecond < 0)
            return;

        if (linearAccelMag > LINEAR_ACCEL_CONSTRAINT) {
            accelX *= Math.abs(LINEAR_ACCEL_CONSTRAINT / linearAccelMag);
            accelY *= Math.abs(LINEAR_ACCEL_CONSTRAINT / linearAccelMag);

            speeds.vxMetersPerSecond = lastCommandedChassisSpeeds.vxMetersPerSecond + (accelX * dtSeconds);
            speeds.vyMetersPerSecond = lastCommandedChassisSpeeds.vyMetersPerSecond + (accelY * dtSeconds);
        }

        double angularAccel = (speeds.omegaRadiansPerSecond - lastCommandedChassisSpeeds.omegaRadiansPerSecond)
                / dtSeconds;

        if (Math.abs(angularAccel) > ANGULAR_ACCEL_CONSTRAINT) {
            angularAccel = Math.signum(angularAccel) * ANGULAR_ACCEL_CONSTRAINT;
            speeds.omegaRadiansPerSecond = lastCommandedChassisSpeeds.omegaRadiansPerSecond
                    + (angularAccel * dtSeconds);
        }
    }

    public void stopMotion() {
        drive(0, 0, 0, false);
    }

    public void zeroGyro() {
        rotationOffsetRadians = -getRotation().getRadians() - Math.PI / 2;
        // resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(0)));
    }

    // public void zeroAutoGyro() {
    // rotationOffsetRadians = -getRotation().getRadians() - 3 * Math.PI/2 +
    // Math.PI/2;
    // resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(0)));
    // }

    public void runDriveVolts(double voltage) {
        frontLeft.runForward(voltage);
        frontRight.runForward(voltage);
        backLeft.runForward(voltage);
        backRight.runForward(voltage);
    }

    public void runSteerVolts(double voltage) {
        frontLeft.runRotation(voltage);
        frontRight.runRotation(voltage);
        backLeft.runRotation(voltage);
        backRight.runRotation(voltage);
    }

    // Getters
    public SwerveModulePosition[] getSwervePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    // public AHRS getNavx() {
    // return navX;
    // }

    public HolonomicDriveController getDriveController() {
        return driveController;
    }

    /**
     * @return a rotation2d object representing the robot's zeored heading, with 0
     *         degrees being the direction the robot will drive forward in
     */
    public Rotation2d getAdjustedRotation() {
        return getRotation().plus(Rotation2d.fromRadians(rotationOffsetRadians + Math.PI));
    }

    /**
     * @return a rotation2d object representing the robot's current heading, with 0
     *         degrees being the direction the robot was facing at startup
     */
    public Rotation2d getRotation() {
        return navX.getRotation2d().plus(Rotation2d.fromRadians(Math.PI));

    }

    public ChassisSpeeds getChassisSpeeds() {
        return currentChassisSpeeds;
    }

    // public BiConsumer getChassisOutput(ChassisSpeeds chassisSpeed,
    // DriveFeedforwards feedforwards){
    // return null; //Needs to return a biconsumer of chassisspeeds and feedforward
    // }

    // Setters
    public void setModuleStates(SwerveModuleState[] moduleStates) {
        frontLeft.setDesiredState(moduleStates[0]);// 0
        frontRight.setDesiredState(moduleStates[1]);// 1
        backLeft.setDesiredState(moduleStates[2]);// 2
        backRight.setDesiredState(moduleStates[3]);// 3
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getRotation(), getSwervePositions(), pose);
    }

    public void newAutoResetPose(Pose2d pose) {
        poseEstimator.resetPosition(getRotation(),
                getSwervePositions(),
                new Pose2d(new Translation2d(pose.getX(), pose.getY()),
                        Rotation2d.fromDegrees(179.79)));
    }

    public void setGyro(Rotation2d rotation) {
        rotationOffsetRadians = -getRotation().getRadians() + rotation.getRadians();
    }

    /**
     * Converts raw joystick values to speeds for the drivetrain
     * <p>
     * This method applies deadbands and curves to the joystick values and clamps
     * the resultant speed to the linear velocity constraint
     * 
     * @param xAxis   value fro m -1 to 1 representing the x-axis of the joystick
     * @param yAxis   value from -1 to 1 representing the y-axis of the joystick
     * @param rotAxis value from -1 to 1 representing the rotation axis of the
     *                joystick
     * @return a ChassisSpeeds object representing the speeds to be passed to the
     *         drivetrain
     */

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

    // Commands

    public Command zeroGyroCommand() {
        return Commands.runOnce(this::zeroGyro, this);
    }

    public Command enableSpeedModeCommand() {
        return Commands.runOnce(() -> {
            speedMode = true;
        });
    }

    public Command disableSpeedModeCommand() {
        return Commands.runOnce(() -> {
            speedMode = false;
            brakeModeScale = 0;
        });
    }

    public void transferBrakeMode() {
        if (speedMode && brakeModeScale < 1) {
            brakeModeScale = Math.min(1, brakeModeScale + 0.02);
        }
    }

    public Command joystickDriveCommand(DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rotAxis) {
        return Commands.run(() -> {
            double oX = xAxis.getAsDouble();
            double oY = yAxis.getAsDouble();
            double oR = rotAxis.getAsDouble();

            brakeModeScale = Math.min(1, brakeModeScale); // Leave this here!
            double x = oX * brakeModeScale + oX * 0.35 * (1 - brakeModeScale);
            double y = oY * brakeModeScale + oY * 0.35 * (1 - brakeModeScale);
            double r = oR * brakeModeScale + oR * 0.50 * (brakeModeScale);

            ChassisSpeeds speeds = joystickAxesToChassisSpeeds(x, y, r);
            drive(speeds, true);
        }, this);
    }

    public Command driveSysIdRoutineCommand() {
        return Commands.sequence(
                driveSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(4),
                Commands.waitSeconds(1),
                driveSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(4),
                Commands.waitSeconds(1),
                driveSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(1),
                Commands.waitSeconds(1),
                driveSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(1),
                Commands.waitSeconds(1));
    }

    public Command steerSysIdRoutineCommand() {
        return Commands.sequence(
                steerSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(7),
                Commands.waitSeconds(1),
                steerSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(7),
                Commands.waitSeconds(1),
                steerSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(2),
                Commands.waitSeconds(1),
                steerSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(2),
                Commands.waitSeconds(1));
    }

    // public Command followPathCommand(String pathName) throws IOException,
    // ParseException{
    // PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    // return new FollowPathCommand(
    // path,
    // this::getPose,
    // this::getChassisSpeeds,
    // null, //needs to be replaced with actual biconsumer
    // HOLONOMIC_PATH_FOLLOWER_CONFIG,
    // config,
    // ON_RED_ALLIANCE,
    // this
    // );
    // }

    public void timedDriveCommand(double xSpeed, double ySpeed, double angularVelocity, boolean fieldRelative,
            double driveTime) {
        new SequentialCommandGroup(
                Commands.run(() -> drive(xSpeed, ySpeed, angularVelocity, fieldRelative), this).withTimeout(driveTime),
                Commands.runOnce(() -> stopMotion())).schedule();
    }
}
