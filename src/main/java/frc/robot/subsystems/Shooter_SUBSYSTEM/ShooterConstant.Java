package frc.robot.subsystems.shooter;

/**
 * Stores all constants used by the shooter subsystem.
 * 
 * This includes motor IDs, PID gains, and feedforward constants.
 * If we ever switch ports or retune values, we only have to change them here.
 */
public final class ShooterConstants {

    // === Motor IDs ===
    public static final int LEFT_MOTOR_ID = 1;
    public static final int RIGHT_MOTOR_ID = 2;

    // === PID Gains ===
    // Start with small numbers; tune later in testing.
    public static final double kP = 0.0005;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // === Feedforward Gains ===
    // These are rough guesses — tune during shooting tests.
    public static final double kS = 0.2;   // static friction
    public static final double kV = 0.12;  // velocity
    public static final double kA = 0.015; // acceleration

    // === Target Shooter Speeds (RPM) ===
    public static final double CLOSE_SHOT_RPM = 2500;
    public static final double FAR_SHOT_RPM = 4000;

    private ShooterConstants() {
        // Prevent instantiation
    }
}