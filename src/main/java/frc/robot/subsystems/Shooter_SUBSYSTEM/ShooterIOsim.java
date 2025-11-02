public package frc.robot.subsystems.shooter;

/**
 * Simulated version of the ShooterIO.
 * 
 * This class doesn’t control real motors — it just pretends to,
 * so we can test code without hardware. 
 * For now, it mostly prints or stores values so we can see what would happen.
 */
public class ShooterIOsim implements ShooterIO {

    private double currentRPM = 0.0;
    private double appliedVoltage = 0.0;

    private double kS = 0.0;
    private double kV = 0.0;
    private double kA = 0.0;

    private double kP = 0.0;
    private double kI = 0.0;
    private double kD = 0.0;

    @Override
    public void setRPM(double rpm) {
        currentRPM = rpm;
        System.out.println("[Sim] Shooter target RPM set to: " + rpm);
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVoltage = voltage;
        System.out.println("[Sim] Shooter voltage set to: " + voltage + "V");
    }

    @Override
    public void setFeedforwardGains(double Ks, double Kv, double Ka) {
        this.kS = Ks;
        this.kV = Kv;
        this.kA = Ka;
        System.out.println("[Sim] Feedforward gains set → Ks: " + Ks + ", Kv: " + Kv + ", Ka: " + Ka);
    }

    @Override
    public void setPIDGains(double Kp, double Ki, double Kd) {
        this.kP = Kp;
        this.kI = Ki;
        this.kD = Kd;
        System.out.println("[Sim] PID gains set → Kp: " + Kp + ", Ki: " + Ki + ", Kd: " + Kd);
    }

    /**
     * Optional helper to view what the simulation thinks the current state is.
     */
    public void printSimStatus() {
        System.out.println("[Sim Status] RPM: " + currentRPM + " | Voltage: " + appliedVoltage);
    }
}
 {
    
}
