package frc.robot.subsystems.shooter;

/**
 * ShooterIO acts as a low-level interface for the shooter hardware.
 * 
 * Basically, this is what the subsystem talks to in order to control
 * the actual shooter motors. The real motor code (Spark MAX, etc.)
 * would go in a class that implements this interface.
 */
public interface ShooterIO {

    /** 
     * Sets the target shooter speed in RPM.
     * The subsystem will call this whenever it wants the shooter to spin
     * at a certain speed.
     */
    default void setRPM(double rpm) {
        // Left empty on purpose — the real implementation will fill this in.
    }

    /**
     * Directly sets the motor voltage. This is used for open-loop control.
     */
    default void setVoltage(double voltage) {
        // Left empty — real motor code goes here later.
    }

    /**
     * Sets the feedforward gains (Ks, Kv, Ka).
     * These are constants used to estimate how much voltage to apply
     * for a given speed and acceleration.
     */
    default void setFeedforwardGains(double Ks, double Kv, double Ka) {
        // Nothing yet — just defines the method for the real IO class.
    }

    /**
     * Sets the PID controller gains (Kp, Ki, Kd).
     * These control how the shooter reacts to error in RPM.
     */
    default void setPIDGains(double Kp, double Ki, double Kd) {
        // No implementation here — just a placeholder for now.
    }
}
