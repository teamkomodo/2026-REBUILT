package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkMax;

public final class Constants {

  public static final boolean TUNING_MODE = false;

  // Controls
  public static final double XBOX_DEADBAND = 0.06;
  public static final int DRIVER_XBOX_PORT = 0;
  public static final int OPERATOR_XBOX_PORT = 1;

  public static final double LINEAR_SLOW_MODE_MODIFIER = 0.5;
  public static final double ANGULAR_SLOW_MODE_MODIFIER = 0.3;
  public static final double DRIVETRAIN_WIDTH = 0.57785; // Distance between center of left and right swerve wheels in
                                                         // meters
  public static final double DRIVETRAIN_LENGTH = 0.57785; // Distance between center of front and back swerve wheels in
                                                          // meters

  public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 20;
  public static final int BACK_RIGHT_STEER_MOTOR_ID = 24;
  public static final int BACK_RIGHT_STEER_ENCODER_ID = 23;
  public static final double BACK_RIGHT_STEER_OFFSET = 5.277;

  public static final int BACK_LEFT_DRIVE_MOTOR_ID = 11;
  public static final int BACK_LEFT_STEER_MOTOR_ID = 14;
  public static final int BACK_LEFT_STEER_ENCODER_ID = 22;
  public static final double BACK_LEFT_STEER_OFFSET = 1.089;

  public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 12;
  public static final int FRONT_RIGHT_STEER_MOTOR_ID = 23;
  public static final int FRONT_RIGHT_STEER_ENCODER_ID = 21;
  public static final double FRONT_RIGHT_STEER_OFFSET = 1.2796;

  public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 36;
  public static final int FRONT_LEFT_STEER_MOTOR_ID = 21;
  public static final int FRONT_LEFT_STEER_ENCODER_ID = 20;
  public static final double FRONT_LEFT_STEER_OFFSET = 1.7006;

  public static final double WHEEL_DIAMETER = 0.1016;

  // unit conversions
  public static final double INCHES_PER_METER = 39.37;

  // motor rotations -> wheel rotations
  public static final double DRIVE_REDUCTION = 1 / 6.75; // (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)
  // motor rotations -> module rotations
  public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);

  public static double MAX_ATTAINABLE_VELOCITY = 4.5;

  public static final double LINEAR_VELOCITY_CONSTRAINT = MAX_ATTAINABLE_VELOCITY;
  public static final double LINEAR_ACCEL_CONSTRAINT = 12.0;

  public static final double ANGULAR_VELOCITY_CONSTRAINT = (LINEAR_VELOCITY_CONSTRAINT * Math.PI)
      / (DRIVETRAIN_WIDTH * DRIVETRAIN_WIDTH + DRIVETRAIN_LENGTH * DRIVETRAIN_LENGTH) * 0.8;
  public static final double ANGULAR_ACCEL_CONSTRAINT = (LINEAR_ACCEL_CONSTRAINT * Math.PI)
      / (DRIVETRAIN_WIDTH * DRIVETRAIN_WIDTH + DRIVETRAIN_LENGTH * DRIVETRAIN_LENGTH) * 0.5;

  public static final boolean FIELD_RELATIVE_DRIVE = true;
  public static final boolean ALIGNMENT_DRIVE = false;

  public static final double MAX_MODULE_VELOCITY = 4.058; // physical maximum attainable speed of swerve modules
  public static final double MAX_MODULE_ACCEL = 21; // physical maximum attainable accel of swerve modules

  public static final double MAX_ANGULAR_VELOCITY = 4.0 * Math.PI; // constraint for angular velocity
  public static final double MAX_ANGULAR_ACCEL = 4.0 * Math.PI; // constraint for angular acceleration

  public static final PIDConstants DRIVE_PID = new PIDConstants(2, 0, 0);
  public static final PIDConstants STEER_PID = new PIDConstants(1, 0, 0.0002);

  // Shooter
  // The motors on the right side have a CAN id ending in a 2, left side end in 1
  public static final int SHOOTER_MOTOR_RIGHT_ID = 54;
  public static final int SHOOTER_MOTOR_LEFT_ID = 31;
  public static final int SHOOTER_SMART_CURRENT_LIMIT = 30; // FIXME
  public static final double SHOOTER_GEAR_RATIO = 1.5; // 1:1.5 speeding up the flywheel
  public static final double MAX_SHOOTER_SPEED_TOLERANCE = 50.0; // RPM tolerance for considering shooter at target
                                                                 // speed

  // Interestingly, shooter controls the feeder motor, while indexer has the
  // beambreaks
  // Shooter feeder (ball feeding) constants
  // Feeder motors: lead (has encoder/controller) and follower
  public static final int SHOOTER_FEEDER_MOTOR_RIGHT_ID = 52;
  public static final int SHOOTER_FEEDER_MOTOR_LEFT_ID = 51;
  public static final double SHOOTER_FEEDER_ROTATIONS_PER_BALL = 1.0; // FIXME: tune this
  public static final int SHOOTER_MAIN_INVERSION = -1; // Shooter motor is inverted to achieve correct direction
  public static final double SHOOTER_FEEDER_FEED_SPEED = 0.25; // FIXME: tune this
  public static final int SHOOTER_FEEDER_SMART_CURRENT_LIMIT = 30; // FIXME: Make sure this is a good value
  public static final double BEAMBREAK_DEBOUNCE_DURATION = 2.5; // A delay for bouncy balls

  // Shooter physics simulation constants
  public static final double GRAVITY = 9.80665;
  public static final double AIR_DENSITY = 1.14; // Approximate air density in CO Boulder
  public static final double DRAG_COEFFICIENT = 0.45; // Drag coefficient for sphere
  public static final double BALL_RADIUS = (5.91 / INCHES_PER_METER) / 2.0; // 5.91 inch diameter ball
  public static final double BALL_AREA = Math.PI * Math.pow(BALL_RADIUS, 2); // Cross-sectional area of the ball
  public static final double BALL_MASS = 0.215; // Mass of FUEL ball in kg
  public static final double DELTA_TIME = 0.002; // Increased precision for validation
  public static final double LAUNCH_ANGLE = 70.0;
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
  public static final double SHORT_BASELINE_RPM = 3000;
  public static final double LONG_BASELINE_RPM = 4500;
  public static final double PASS_SHOT_RPM = 5500;

  // Finished table
  // These values are pre-generated using the ShooterTableGenerator class in
  // ShooterSubsystem
  // Robot cannot get more than sqrt((317.7/2-27/2)**2+(158.6-27/2)**2) / 39.37 =
  // ~5.22m away, so this table is sufficently large.
  public static final double[] SHOOTER_DISTANCES = { 0.760, 0.910, 1.060, 1.210, 1.360, 1.510, 1.660, 1.810, 1.960,
      2.110, 2.260, 2.410, 2.560, 2.710, 2.860, 3.010, 3.160, 3.310, 3.460, 3.610, 3.760, 3.910, 4.060, 4.210, 4.360,
      4.510, 4.660, 4.810, 4.960, 5.110, 5.260, 5.410, 5.560, 5.710, 5.860, 6.010, 6.160, 6.310, 6.460, 6.610, 6.760,
      6.910, 7.060, 7.210, 7.360, 7.510, 7.660, 7.810, 7.960 };
  public static final double[] SHOOTER_RPMS = { 2249.560, 2144.172, 2138.794, 2171.941, 2222.114, 2282.167, 2345.449,
      2412.512, 2479.890, 2549.671, 2617.397, 2687.568, 2755.280, 2824.306, 2890.317, 2957.447, 3025.737, 3091.615,
      3158.406, 3222.482, 3287.518, 3352.802, 3418.573, 3482.073, 3545.718, 3609.897, 3671.989, 3735.946, 3797.731,
      3859.944, 3922.575, 3985.612, 4049.056, 4109.084, 4173.102, 4233.650, 4295.919, 4359.141, 4421.428, 4483.337,
      4545.577, 4607.759, 4669.065, 4730.113, 4793.585, 4857.450, 4919.507, 4981.861, 5046.699 };

  // Intake
  public static final int INTAKE_MOTOR_LEFT_ID = 41;
  public static final int INTAKE_SMART_CURRENT_LIMIT = 30; // FIXME

  public static final double INTAKE_INTAKE_SPEED = 0.3; // FIXME
  public static final double INTAKE_FEED_SPEED = 0.2; // FIXME
  public static final double INTAKE_EJECT_SPEED = -0.3; // FIXME

  public static final double INTAKE_EJECT_TIME = 0.3; // FIXME
  public static final double INTAKE_STOWING_SPEED = 0.1; // FIXME

  // Hinge
  // Intake hinge uses a single motor (right-side convention kept historically)
  public static final int HINGE_MOTOR_ID = 44;

  public static final double HINGE_STOW_POSITION = 0.0; // FIXME
  public static final double HINGE_FEED_POSITION = 0.0; // FIXME
  public static final double HINGE_INTAKE_POSITION = 0.0; // FIXME
  public static final double HINGE_EJECT_POSITION = 0.0; // FIXME

  // Indexer
  public static final int INDEXER_MOTOR_ID = 43;
  public static final int INDEXER_SMART_CURRENT_LIMIT = 30; // FIXME

  public static final double INDEXER_SPEED = 0.3; // FIXME
  // Indexer beam-break sensor DIO channels (update to real wiring)
  public static final int INDEXER_BEAM_BREAK_FULL_CHANNEL = 0; // Indexer is full sensor //FIXME
  public static final int INDEXER_BEAM_BREAK_READY_CHANNEL = 1; // Indexer is empty sensor //FIXME

  // Basic constants
  public static final SparkMax.MotorType BRUSHLESS = SparkMax.MotorType.kBrushless;
  public static PPHolonomicDriveController HOLONOMIC_PATH_FOLLOWER_CONFIG = new PPHolonomicDriveController(
      DRIVE_PID, // Translation Constants
      STEER_PID // Steering Constants
  );
}