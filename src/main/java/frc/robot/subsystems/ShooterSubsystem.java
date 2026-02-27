package frc.robot.subsystems;

import static frc.robot.Constants.*;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PIDGains;

/**
 * Small command helpers to integrate with SystemStateMachine expectations.
 */

public class ShooterSubsystem extends SubsystemBase {
    // Main networkTable
    private final NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("shooter");

    // Shooter telementry
    private final DoublePublisher shooterSpeedPublisher = shooterTable.getDoubleTopic("shooter-speed").publish();
    private final DoublePublisher shooterRpmPublisher = shooterTable.getDoubleTopic("shooter-rpm").publish();
    private final DoublePublisher shooterDesiredSpeedPublisher = shooterTable.getDoubleTopic("shooter-desired-speed")
            .publish();

    // Feeder telemetry
    private final DoublePublisher feederSpeedPublisher = shooterTable.getDoubleTopic("feeder-speed").publish();
    private final DoublePublisher feederRpmPublisher = shooterTable.getDoubleTopic("feeder-rpm").publish();
    private final DoublePublisher feederDesiredSpeedPublisher = shooterTable.getDoubleTopic("feeder-desired-speed")
            .publish();

    private final SparkFlex shooterMotorRight;
    private final SparkFlex shooterMotorLeft;
    private final SparkFlexConfig shooterMotorRightConfig;
    private final SparkFlexConfig shooterMotorLeftConfig;

    // Feeder motors that move balls into the flywheel (lead + follower)
    private final SparkFlex feederRightMotor;
    private final SparkFlex feederLeftMotor;
    private final SparkFlexConfig feederRightMotorConfig;
    private final SparkFlexConfig feederLeftMotorConfig;

    private final SparkClosedLoopController shooterMotorRightController;
    private final RelativeEncoder shooterMotorRightRelativeEncoder;
    private final PIDGains shooterPidGains;

    private final SparkClosedLoopController feederController;
    private final RelativeEncoder feederEncoder;
    private final PIDGains feederPidGains;

    private double desiredSpeed; // RPMs
    private double desiredFeederSpeed; // duty cycle for feeder (telemetry)

    public ShooterSubsystem() {

        shooterMotorRight = new SparkFlex(SHOOTER_MOTOR_RIGHT_ID, BRUSHLESS);
        shooterMotorLeft = new SparkFlex(SHOOTER_MOTOR_LEFT_ID, BRUSHLESS);
        shooterMotorRightConfig = new SparkFlexConfig();
        shooterMotorLeftConfig = new SparkFlexConfig();

        // Lead feeder (contains encoder & controller), follower mirrors the lead
        feederRightMotor = new SparkFlex(Constants.SHOOTER_FEEDER_MOTOR_RIGHT_ID, BRUSHLESS);
        feederLeftMotor = new SparkFlex(Constants.SHOOTER_FEEDER_MOTOR_LEFT_ID, BRUSHLESS);
        feederRightMotorConfig = new SparkFlexConfig();
        feederLeftMotorConfig = new SparkFlexConfig();

        shooterMotorRightController = shooterMotorRight.getClosedLoopController();
        shooterMotorRightRelativeEncoder = shooterMotorRight.getEncoder();
        shooterPidGains = new PIDGains(0.0001, 0.0, 0.0, 0.0001); // FIXME: tune these; Add d for faster comeback

        feederController = feederRightMotor.getClosedLoopController();
        feederEncoder = feederRightMotor.getEncoder();
        feederPidGains = new PIDGains(1.0, 0.0, 0.0, 0.0); // FIXME: Tune pid constants

        desiredSpeed = 0.0;
        desiredFeederSpeed = 0.0;
        configureMotors();
    }

    public void teleopInit() {
    }

    @Override
    public void periodic() {
        updateTelemetry();
    }

    public void configureMotors() {
        shooterMotorRightConfig
                .smartCurrentLimit(SHOOTER_SMART_CURRENT_LIMIT)
                .idleMode(IdleMode.kCoast)
                .inverted(false);

        shooterMotorRightConfig.closedLoop
                .p(shooterPidGains.p)
                .i(shooterPidGains.i)
                .d(shooterPidGains.d);

        shooterMotorRight.configure(
                shooterMotorRightConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        shooterMotorLeftConfig
                .smartCurrentLimit(SHOOTER_SMART_CURRENT_LIMIT)
                .follow(SHOOTER_MOTOR_RIGHT_ID, true)
                .idleMode(IdleMode.kCoast);

        shooterMotorLeft.configure(
                shooterMotorLeftConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Configure feeder motor
        // Lead feeder closed-loop configuration (encoder/controller on lead)
        feederRightMotorConfig.closedLoop
                .p(feederPidGains.p)
                .i(feederPidGains.i)
                .d(feederPidGains.d);

        feederRightMotorConfig
                .smartCurrentLimit(SHOOTER_FEEDER_SMART_CURRENT_LIMIT)
                .idleMode(IdleMode.kBrake);

        feederRightMotor.configure(
                feederRightMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Configure follower to mirror the lead (no inversion)
        feederLeftMotorConfig
                .follow(Constants.SHOOTER_FEEDER_MOTOR_RIGHT_ID, true)
                .smartCurrentLimit(SHOOTER_FEEDER_SMART_CURRENT_LIMIT)
                .idleMode(IdleMode.kBrake);

        feederLeftMotor.configure(
                feederLeftMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void updateTelemetry() {
        shooterSpeedPublisher.set(shooterMotorRight.getAppliedOutput());
        shooterRpmPublisher.set(shooterMotorRightRelativeEncoder.getVelocity());
        shooterDesiredSpeedPublisher.set(desiredSpeed);

        // Feeder telemetry
        feederSpeedPublisher.set(feederRightMotor.getAppliedOutput());
        feederRpmPublisher.set(feederEncoder.getVelocity());
        feederDesiredSpeedPublisher.set(desiredFeederSpeed);
    }

    public void setShooterDutyCycle(double dutyCycle) {
        shooterMotorRightController.setSetpoint(dutyCycle * SHOOTER_MAIN_INVERSION, ControlType.kDutyCycle);
    }

    public void setShooterVelocity(double velocity) {
        shooterMotorRightController.setSetpoint(velocity * SHOOTER_MAIN_INVERSION, ControlType.kVelocity);
    }

    public void stopShooter() {
        setShooterDutyCycle(0.0);
    }

    public Command stopShooterCommand() {
        return Commands.runOnce(this::stopShooter, this);
    }

    // --- Feeder controls -------------------------------------------------

    public void updateFeederDutyCycle(double dutycycle) {
        desiredFeederSpeed = dutycycle;
        setFeederDutyCycle(dutycycle);
    }

    /** Start continuous feeding (runs at configured feed speed). */
    public void startFeeding() {
        updateFeederDutyCycle(SHOOTER_FEEDER_FEED_SPEED);
    }

    /** Stop the feeder motor. */
    public void stopFeeding() {
        updateFeederDutyCycle(0.0);
    }

    /** Advance the feeder by the configured rotations-per-ball. */
    public void feedOnce() {
        double target = feederEncoder.getPosition() + SHOOTER_FEEDER_ROTATIONS_PER_BALL;
        feederController.setSetpoint(target, ControlType.kPosition);
    }

    public Command feedOnceCommand() {
        return Commands.runOnce(this::feedOnce, this);
    }

    public Command startFeedingCommand() {
        // This command will set the feeder to run at feed speed while scheduled.
        return Commands.runOnce(this::startFeeding, this);
    }

    public Command stopFeedingCommand() {
        return Commands.runOnce(this::stopFeeding, this);
    }

    public void setFeederDutyCycle(double dutyCycle) {
        System.out.println("========================= Starting feeder: " + dutyCycle);
        feederController.setSetpoint(dutyCycle, ControlType.kDutyCycle);
    }

    public void holdShooter() {
        shooterMotorRightController.setSetpoint(shooterMotorRightRelativeEncoder.getPosition(), ControlType.kPosition);
    }

    public Command updateShooterSpeed(double desiredSpeed) {
        this.desiredSpeed = desiredSpeed;
        return Commands.runOnce(() -> setShooterVelocity(this.desiredSpeed), this);
    }

    public Command updateFlywheelSpeedRPM(double desiredFlywheelSpeed) {
        double targetSpeed = desiredFlywheelSpeed / Constants.SHOOTER_GEAR_RATIO;
        System.out.println("-----------------UPDATING SHOOTER RPM: " + desiredFlywheelSpeed);
        return updateShooterSpeed(targetSpeed);
    }

    public boolean isAtTargetSpeed() {
        return Math.abs(shooterMotorRightRelativeEncoder.getVelocity() - desiredSpeed) < MAX_SHOOTER_SPEED_TOLERANCE;
    }

    public Command startShootingCommand() {
        double distanceToHub = 4.0; // 4m is a placeholder for now; FIXME: Replace placeholder
        /*
         * FIXME: need to call out to navx for this
         * Ask @Bora A
         * TODO: Do I need to consider increasing distance by ball radius to account
         * for a potential offset between limelight and shooter exit center?
         */
        return updateShooterSpeed(ShooterLookupTable.findShooterSpeed(distanceToHub));
    }

    public Command shortShotCommand() {
        return updateFlywheelSpeedRPM(SHORT_BASELINE_RPM);
    }

    public Command longShotCommand() {
        return updateFlywheelSpeedRPM(LONG_BASELINE_RPM);
    }

    public Command passShotCommand() {
        return updateFlywheelSpeedRPM(PASS_SHOT_RPM);
    }

    public Command stowShooterCommand() {
        // default stow behavior: stop shooter for now
        return stopShooterCommand();
    }

    // Shooting calculation:
    public class ShooterLookupTable {
        // Distance in meters, RPM in rotations per minute

        public static double findShooterSpeed(double distance) {
            // 1. Handle Out-of-Bounds
            if (distance <= SHOOTER_DISTANCES[0])
                return SHOOTER_RPMS[0];
            if (distance >= SHOOTER_DISTANCES[SHOOTER_DISTANCES.length - 1])
                return SHOOTER_RPMS[SHOOTER_RPMS.length - 1];

            // 2. Find the bounding indices (Linear Search)
            int i = 0;
            while (SHOOTER_DISTANCES[i + 1] < distance) {
                i++;
            }

            // 3. Linear Interpolation
            // Formula: y = y0 + (x - x0) * ((y1 - y0) / (x1 - x0))
            double x0 = SHOOTER_DISTANCES[i];
            double x1 = SHOOTER_DISTANCES[i + 1];
            double y0 = SHOOTER_RPMS[i];
            double y1 = SHOOTER_RPMS[i + 1];

            double calculatedRPM = y0 + (distance - x0) * ((y1 - y0) / (x1 - x0));
            // Limit flywheel speed
            return Math.min(calculatedRPM, MAX_FLYWHEEL_RPM);
        }
    }

    public class ShooterTableGenerator {
        // --- Physics Constants in Constants file ---

        // --- Table Generation Settings ---
        private static final double MIN_DIST = 0.76; // Meters
        private static final double MAX_DIST = 8.0; // Meters
        private static final double STEP_SIZE = 0.15; // Meters (Adjustable)
        private static final double MAX_ERROR_METERS = 10.0 / Constants.INCHES_PER_METER; // 0.254m

        // The main method is commented out to prevent accidental execution. Uncomment
        // to generate table and print results.
        // public static void main(String[] args) {
        // createTable();
        // }

        // Main function; called in main() method
        public static void createTable() {
            List<Double> distances = new ArrayList<>();
            List<Double> rpms = new ArrayList<>();

            // 1. Generate Table
            for (double d = MIN_DIST; d <= MAX_DIST; d += STEP_SIZE) {
                distances.add(d);
                rpms.add(calculateFlywheelRPM(findVelocity(d, HUB_OPENING_HEIGHT, LAUNCH_ANGLE)));
            }

            // 2. Validate Table (Check midpoints for linear interpolation error)
            System.out.println("--- Validation Report ---");
            boolean passed = true;
            double maxErrorFound = 0;

            for (int i = 0; i < distances.size() - 1; i++) {
                double dMid = (distances.get(i) + distances.get(i + 1)) / 2.0;
                double rpmMid = (rpms.get(i) + rpms.get(i + 1)) / 2.0;

                // Where does this interpolated RPM actually land?
                double actualX = simulateForX(calculateV0FromRPM(rpmMid), LAUNCH_ANGLE,
                        HUB_OPENING_HEIGHT);
                double error = Math.abs(actualX - dMid);

                if (error > maxErrorFound)
                    maxErrorFound = error;
                if (error > MAX_ERROR_METERS) {
                    passed = false;
                    System.out.printf("FAIL at %.2fm: Error is %.3f inches%n", dMid, error /
                            0.0254);
                }
            }

            System.out.printf("Validation Status: %s%n", passed ? "PASSED" : "FAILED");
            System.out.printf("Max Distance Error: %.2f inches%n%n", maxErrorFound /
                    0.0254);

            // 3. Print Final Code-Ready Table
            System.out.println("--- Final Lookup Table (Java Arrays) ---");
            printArray("SHOOTER_DISTANCES", distances);
            printArray("SHOOTER_RPMS", rpms);
        }

        // --- Core Logic ---

        public static double calculateFlywheelRPM(double v0) {
            double surfaceSpeed = v0 / FLYWHEEL_SURFACE_TO_BALL_SPEED_RATIO;
            return ((surfaceSpeed / FLYWHEEL_RADIUS) * 60.0) / (2.0 * Math.PI);
        }

        public static double calculateV0FromRPM(double rpm) {
            double omega = (rpm * 2.0 * Math.PI) / 60.0;
            double surfaceSpeed = omega * FLYWHEEL_RADIUS;
            return surfaceSpeed * FLYWHEEL_SURFACE_TO_BALL_SPEED_RATIO;
        }

        public static double findVelocity(double tx, double ty, double angle) {
            double lowV = 0.0, highV = 100.0;
            double bestV = 0.0;
            for (int i = 0; i < 25; i++) {
                double midV = (lowV + highV) / 2.0;
                if (simulateAtDistance(midV, angle, tx) < ty)
                    lowV = midV;
                else
                    highV = midV;
                bestV = midV;
            }
            return bestV;
        }

        /** Returns Y-height at a specific X-distance */
        private static double simulateAtDistance(double v0, double theta, double targetX) {
            double rad = Math.toRadians(theta);
            double vx = v0 * Math.cos(rad), vy = v0 * Math.sin(rad);
            double x = 0, y = 0, vSpin = v0;

            while (x < targetX) {
                double v = Math.sqrt(vx * vx + vy * vy);
                double Cl = LIFT_COEFFICIENT * (vSpin / v);
                double drag = 0.5 * AIR_DENSITY * v * v * DRAG_COEFFICIENT * BALL_AREA;
                double magnus = 0.5 * AIR_DENSITY * v * v * Cl * BALL_AREA;
                double ax = -(drag * (vx / v) + magnus * (vy / v)) / BALL_MASS;
                double ay = (-GRAVITY * BALL_MASS - drag * (vy / v) + magnus * (vx / v)) / BALL_MASS;
                vx += ax * DELTA_TIME;
                vy += ay * DELTA_TIME;
                x += vx * DELTA_TIME;
                y += vy * DELTA_TIME;
                if (y < -1.0)
                    break;
            }
            return y;
        }

        /** Returns X-distance when ball falls back to target height (Validation use) */
        private static double simulateForX(double v0, double theta, double targetY) {
            double rad = Math.toRadians(theta);
            double vx = v0 * Math.cos(rad), vy = v0 * Math.sin(rad);
            double x = 0, y = 0, vSpin = v0;
            boolean peaked = false;

            while (true) {
                double v = Math.sqrt(vx * vx + vy * vy);
                double Cl = 0.15 * (vSpin / v);
                double drag = 0.5 * AIR_DENSITY * v * v * DRAG_COEFFICIENT * BALL_AREA;
                double magnus = 0.5 * AIR_DENSITY * v * v * Cl * BALL_AREA;
                vx += (-(drag * (vx / v) + magnus * (vy / v)) / BALL_MASS) * DELTA_TIME;
                vy += ((-GRAVITY * BALL_MASS - drag * (vy / v) + magnus * (vx / v)) / BALL_MASS) * DELTA_TIME;
                x += vx * DELTA_TIME;
                y += vy * DELTA_TIME;

                if (vy < 0)
                    peaked = true;
                if (peaked && y <= targetY)
                    return x; // Ball crossed target height on way down
                if (y < -1.0 || x > 15.0)
                    return x;
            }
        }

        private static void printArray(String name, List<Double> vals) {
            System.out.print("  public static final double[] " + name + " = { ");
            for (int i = 0; i < vals.size(); i++) {
                System.out.printf("%.3f%s", vals.get(i), (i == vals.size() - 1) ? "" : ", ");
            }
            System.out.println(" };");
        }
    }

}
