package frc.robot;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
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
  public static final double DRIVETRAIN_WIDTH = 0.57785; // Distance between center of left and right swerve wheels in meters
  public static final double DRIVETRAIN_LENGTH = 0.57785; // Distance between center of front and back swerve wheels in meters
  
  public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 32;
  public static final int BACK_RIGHT_STEER_MOTOR_ID = 34;
  public static final int BACK_RIGHT_STEER_ENCODER_ID = 22;
  public static final double BACK_RIGHT_STEER_OFFSET = 5.277;

  public static final int BACK_LEFT_DRIVE_MOTOR_ID = 39;
  public static final int BACK_LEFT_STEER_MOTOR_ID = 38;
  public static final int BACK_LEFT_STEER_ENCODER_ID = 21;   
  public static final double BACK_LEFT_STEER_OFFSET = 1.089;

  public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 33;
  public static final int FRONT_RIGHT_STEER_MOTOR_ID = 35;
  public static final int FRONT_RIGHT_STEER_ENCODER_ID = 20;  
  public static final double FRONT_RIGHT_STEER_OFFSET = 1.2796;

  public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 36;
  public static final int FRONT_LEFT_STEER_MOTOR_ID = 37;
  public static final int FONT_LEFT_STEER_ENCODER_ID = 23;
  public static final double FRONT_LEFT_STEER_OFFSET = 1.7006;

  public static final double WHEEL_DIAMETER = 0.1016;

  //motor rotations -> wheel rotations
  public static final double DRIVE_REDUCTION = 1 / 6.75; //(14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)
  //motor rotations -> module rotations
  public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);

  public static double MAX_ATTAINABLE_VELOCITY = 4.5;

  public static final double LINEAR_VELOCITY_CONSTRAINT = MAX_ATTAINABLE_VELOCITY;
  public static final double LINEAR_ACCEL_CONSTRAINT = 12.0;

  public static final double ANGULAR_VELOCITY_CONSTRAINT = (LINEAR_VELOCITY_CONSTRAINT * Math.PI) / (DRIVETRAIN_WIDTH * DRIVETRAIN_WIDTH + DRIVETRAIN_LENGTH * DRIVETRAIN_LENGTH) * 0.8;
  public static final double ANGULAR_ACCEL_CONSTRAINT = (LINEAR_ACCEL_CONSTRAINT * Math.PI) / (DRIVETRAIN_WIDTH * DRIVETRAIN_WIDTH + DRIVETRAIN_LENGTH * DRIVETRAIN_LENGTH) * 0.5;

  public static final boolean FIELD_RELATIVE_DRIVE = true;
  public static final boolean ALIGNMENT_DRIVE = false;
  public static final int FRONT_LEFT_STEER_ENCODER_ID = 10;

  public static final double MAX_MODULE_VELOCITY = 4.058; // physical maximum attainable speed of swerve modules
  public static final double MAX_MODULE_ACCEL = 21; // physical maximum attainable accel of swerve modules

  public static final double MAX_ANGULAR_VELOCITY = 4.0 * Math.PI; // constraint for angular velocity
  public static final double MAX_ANGULAR_ACCEL = 4.0 * Math.PI; // constraint for angular acceleration

  public static final PIDConstants DRIVE_PID = new PIDConstants(2, 0, 0);
  public static final PIDConstants STEER_PID = new PIDConstants(1, 0, 0.0002);

  //Shooter
  public static final int SHOOTER_MOTOR_RIGHT_ID = 0; //FIXME
  public static final int SHOOTER_MOTOR_LEFT_ID = 0; //FIXME

  public static final int SHOOTER_SMART_CURRENT_LIMIT = 30; //FIXME

  //Intake
  public static final int INTAKE_MOTOR_RIGHT_ID = 0; //FIXME
  public static final int INTAKE_MOTOR_LEFT_ID = 0; //FIXME

  public static final int INTAKE_SMART_CURRENT_LIMIT = 30; //FIXME

  public static final double INTAKE_INTAKE_SPEED = 0.3; //FIXME
  public static final double INTAKE_FEED_SPEED = 0.2; //FIXME
  public static final double INTAKE_EJECT_SPEED = -0.3; //FIXME

  public static final double INTAKE_EJECT_TIME = 0.3; //FIXME

  //Hinge
  public static final int HINGE_MOTOR_RIGHT_ID = 0; //FIXME
  public static final int HINGE_MOTOR_LEFT_ID = 0; //FIXME

  public static final double HINGE_STOW_POSITION = 0.0; //FIXME
  public static final double HINGE_FEED_POSITION = 0.0; //FIXME
  public static final double HINGE_INTAKE_POSITION = 0.0; //FIXME
  public static final double HINGE_EJECT_POSITION = 0.0; //FIXME

  //Basic constants
  public static final com.revrobotics.spark.SparkLowLevel.MotorType BRUSHLESS = SparkMax.MotorType.kBrushless;
  public static PPHolonomicDriveController HOLONOMIC_PATH_FOLLOWER_CONFIG = new PPHolonomicDriveController(
      DRIVE_PID, // Translation Constants
      STEER_PID // Steering Constants
      );
}