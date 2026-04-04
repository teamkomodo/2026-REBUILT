package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {
    public static final Translation2d RED_HUB_POS_METERS = new Translation2d(16.540 - 4.6255, 4.0345);
    public static final Translation2d BLUE_HUB_POS_METERS = new Translation2d(4.6255, 4.0345);

    public static final boolean TUNING_MODE = false;

    // Controls
    public static final double XBOX_DEADBAND = 0.06;
    public static final int DRIVER_XBOX_PORT = 0;
    public static final int OPERATOR_XBOX_PORT = 1;

    public static final double LINEAR_SLOW_MODE_MODIFIER = 0.5;
    public static final double ANGULAR_SLOW_MODE_MODIFIER = 0.3;
    public static final double DRIVETRAIN_WIDTH = 0.57785; // Distance between center of left and right swerve
                                                           // wheels in
                                                           // meters
    public static final double DRIVETRAIN_LENGTH = 0.57785; // Distance between center of front and back swerve
                                                            // wheels
                                                            // in
                                                            // meters

    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 20;
    public static final int BACK_RIGHT_STEER_MOTOR_ID = 38;
    public static final int BACK_RIGHT_STEER_ENCODER_ID = 22;
    // public static final double BACK_RIGHT_STEER_OFFSET = 5.277 - 0.419 + 0.622;
    public static final double BACK_RIGHT_STEER_OFFSET = 0.55 + Math.PI / 2;

    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 16;
    public static final int BACK_LEFT_STEER_MOTOR_ID = 33;
    public static final int BACK_LEFT_STEER_ENCODER_ID = 21;
    public static final double BACK_LEFT_STEER_OFFSET = -0.407 + Math.PI / 2 + Math.PI;

    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 12;
    public static final int FRONT_RIGHT_STEER_MOTOR_ID = 23;
    public static final int FRONT_RIGHT_STEER_ENCODER_ID = 23;
    public static final double FRONT_RIGHT_STEER_OFFSET = -1.493 + Math.PI / 2 + Math.PI;

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 36;
    public static final int FRONT_LEFT_STEER_MOTOR_ID = 21;
    public static final int FRONT_LEFT_STEER_ENCODER_ID = 20;
    public static final double FRONT_LEFT_STEER_OFFSET = 1.209 + Math.PI / 2 + Math.PI;

    public static final double WHEEL_DIAMETER = 0.1016;

    // unit conversions
    public static final double INCHES_PER_METER = 39.37;

    // motor rotations -> wheel rotations
    public static final double DRIVE_REDUCTION = 1 / 6.75; // (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)
    // motor rotations -> module rotations
    public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);

    public static double MAX_ATTAINABLE_VELOCITY = 6.5; // 6.5

    public static final double LINEAR_VELOCITY_CONSTRAINT = MAX_ATTAINABLE_VELOCITY;
    public static final double LINEAR_ACCEL_CONSTRAINT = 12; // 12 ORGINALL

    public static final double ANGULAR_VELOCITY_CONSTRAINT = (LINEAR_VELOCITY_CONSTRAINT * Math.PI)
            / (DRIVETRAIN_WIDTH * DRIVETRAIN_WIDTH + DRIVETRAIN_LENGTH * DRIVETRAIN_LENGTH) * 1.1; // 0.8
    public static final double ANGULAR_ACCEL_CONSTRAINT = (LINEAR_ACCEL_CONSTRAINT * Math.PI)
            / (DRIVETRAIN_WIDTH * DRIVETRAIN_WIDTH + DRIVETRAIN_LENGTH * DRIVETRAIN_LENGTH) * 0.9; // 0.5

    public static final boolean FIELD_RELATIVE_DRIVE = true;
    public static final boolean ALIGNMENT_DRIVE = false;

    public static final double MAX_MODULE_VELOCITY = 4.058; // physical maximum attainable speed of swerve modules
    public static final double MAX_MODULE_ACCEL = 10; // physical maximum attainable accel of swerve modules

    public static final double MAX_ANGULAR_VELOCITY = 4.0 * Math.PI; // constraint for angular velocity
    public static final double MAX_ANGULAR_ACCEL = 2.0 * Math.PI; // constraint for angular acceleration // 4
                                                                  // ORIGINALS

    public static final PIDConstants DRIVE_PID = new PIDConstants(2, 0, 0);
    public static final PIDConstants STEER_PID = new PIDConstants(1, 0, 0.0002);

    // Shooter
    // The motors on the right side have a CAN id ending in a 2, left side end in 1
    public static final int SHOOTER_MOTOR_RIGHT_ID = 61;
    public static final int SHOOTER_MOTOR_LEFT_ID = 31;
    public static final double SHOOTER_MAX_DUTYCYCLE = 1.0;
    public static final int SHOOTER_MAX_RPM = 10000; // Horrible idea
    public static final int SHOOTER_SMART_CURRENT_LIMIT = 200; // Even worse idea
    public static final double SHOOTER_GEAR_RATIO = 1 / 1.5; // 1:1.5 slowing down the flywheel
    public static final double MAX_SHOOTER_SPEED_TOLERANCE = 50.0; // RPM tolerance for considering shooter at target
                                                                   // speed

    // Interestingly, shooter controls the feeder motor, while indexer has the
    // beambreaks
    // Shooter feeder (ball feeding) constants
    // Feeder motors: lead (has encoder/controller) and follower
    public static final int SHOOTER_FEEDER_MOTOR_RIGHT_ID = 52;
    public static final double SHOOTER_FEEDER_ROTATIONS_PER_BALL = 1.0; // FIXME: tune this
    public static final int SHOOTER_MAIN_INVERSION = -1; // Shooter motor is inverted to achieve correct direction
    public static final double SHOOTER_FEEDER_FEED_SPEED = 1.0; // FIXME: tune this 0.4
    public static final int SHOOTER_FEEDER_SMART_CURRENT_LIMIT = 50; // FIXME: Make sure this is a good value
    public static final double BEAMBREAK_DEBOUNCE_DURATION = 2.5; // A delay for bouncy balls

    // Shooter physics simulation constants
    public static final double GRAVITY = 9.80665;
    public static final double AIR_DENSITY = 1.14; // Approximate air density in CO Boulder
    public static final double DRAG_COEFFICIENT = 0.45; // Drag coefficient for sphere
    public static final double BALL_RADIUS = (5.91 / INCHES_PER_METER) / 2.0; // 5.91 inch diameter ball
    public static final double BALL_AREA = Math.PI * Math.pow(BALL_RADIUS, 2); // Cross-sectional area of the ball
    public static final double BALL_MASS = 0.215; // Mass of FUEL ball in kg
    public static final double DELTA_TIME = 0.002; // Increased precision for validation
    public static final double LAUNCH_ANGLE = 70.0; // Degrees
    public static final double HUB_OPENING_HEIGHT_FROM_GROUND = 72.0 / INCHES_PER_METER;
    public static final double SHOOTER_HEIGHT_FROM_GROUND = (19.5 - 5.91 / 2) / INCHES_PER_METER;
    // ^ Estimated height of the shooter from the ground (ball exit point)
    public static final double HUB_OPENING_HEIGHT = HUB_OPENING_HEIGHT_FROM_GROUND - SHOOTER_HEIGHT_FROM_GROUND;
    // ^ Effective vertical distance the ball must travel
    public static final double FLYWHEEL_RADIUS = 2.0 / INCHES_PER_METER;
    public static final double LIFT_COEFFICIENT = 0.15; // Approximate lift coefficient for backspin (Magnus effect)
    // Since the shooter consists of one wheel pressing a ball against a stationary
    // plate,
    // the ball's initial velocity is half of the surface speed of the wheel.
    public static final double FLYWHEEL_SURFACE_TO_BALL_SPEED_RATIO = 0.5;
    public static final double MAX_FLYWHEEL_RPM = 6700;

    // Manual shot RPMs
    public static final double SHORT_BASELINE_RPM = 9000;/// 4725;
    public static final double LONG_BASELINE_RPM = 3500;// 6300
    public static final double PASS_SHOT_RPM = 2750;

    // Finished table
    // These values are pre-generated using the ShooterTableGenerator class in
    // ShooterSubsystem
    // Robot cannot get more than sqrt((317.7/2-27/2)**2+(158.6-27/2)**2) / 39.37 =
    // ~5.22m away, so this table is sufficently large.
    public static final double[] SHOOTER_DISTANCES_METERS = {
        2.00,
        2.4,
        2.83,
        3.20,
        3.60

    };
    public static final double[] SHOOTER_RPMS = {
        3600,
        3750,
        3800,
        4200,
        4500

    };

    // Intake
    public static final int INTAKE_MOTOR_LEFT_ID = 62;
    public static final int INTAKE_MOTOR_RIGHT_ID = 4;
    public static final int INTAKE_SMART_CURRENT_LIMIT = 50; // FIXME

    public static final double INTAKE_INTAKE_SPEED = 0.9; // FIXME
    public static final double INTAKE_FEED_SPEED = 0.4; // FIXME

    public static final double EVIL_INTAKE_FEED_SPEED = -0.7; // -0.75 for outpost
    public static final double INTAKE_EJECT_SPEED = -0.5; // ` FIXME

    public static final double INTAKE_EJECT_TIME = 0.3; // FIXME
    public static final double INTAKE_STOWING_SPEED = 0.1; // FIXME

    public static final double INTAKE_JIGGLE_FORWARD_DUTYCYCLE = 0.55; // FIXME
    public static final double INTAKE_JIGGLE_REVERSE_DUTYCYCLE = -0.53;
    public static final double INTAKE_JIGGLE_HINGE_LIFT_ROTATIONS = 0.1;
    public static final double INTAKE_GEAR_RATIO = 1.5; // 1:1.5 :: motor:roller

    // Hinge
    // Intake hinge uses a single motor (right-side convention kept historically)
    public static final int HINGE_MOTOR_ID = 44;

    public static final int HINGE_SMART_CURRENT_LIMIT = 80; // FIXME
    public static final double HINGE_DEPLOY_DUTY_CYCLE = -0.3;
    public static final double EVIL_HINGE_DUTY_CYCLE = -0.20;
    public static final double HINGE_STOW_DUTY_CYCLE = 0.6;

    // Basic constants
    public static final SparkMax.MotorType BRUSHLESS = SparkMax.MotorType.kBrushless;
    public static PPHolonomicDriveController HOLONOMIC_PATH_FOLLOWER_CONFIG = new PPHolonomicDriveController(
            DRIVE_PID, // Translation Constants
            STEER_PID // Steering Constants
    );
    public static final BooleanSupplier ON_RED_ALLIANCE = () -> {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    };
}