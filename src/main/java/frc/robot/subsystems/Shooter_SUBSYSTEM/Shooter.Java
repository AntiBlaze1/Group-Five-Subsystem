package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;


public class Shooter extends SubsystemBase {
    private final ShooterIO io;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    /** 
     * Runs the shooter in open loop using a voltage value. 
     * The motor voltage continuously follows the supplier value
     * while the command is active, and stops when it's interrupted.
     */
    public Command runVoltage(DoubleSupplier voltageSupplier) {
        return new RunCommand(
            () -> io.setVoltage(voltageSupplier.getAsDouble()),
            this
        ).finallyDo(interrupted -> io.setVoltage(0));
    }

    /** 
     * Runs the shooter using a target RPM. 
     * The RPM value is read continuously from the supplier, and the shooter
     * stops when the command ends.
     */
    public Command runRPM(DoubleSupplier rpmSupplier) {
        return new RunCommand(
            () -> io.setRPM(rpmSupplier.getAsDouble()),
            this
        ).finallyDo(interrupted -> io.setRPM(0));
    }

    /** 
     * Optional helper methods for direct control (not through Commands).
     * These might be used for autonomous or testing.
     */
    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void setRPM(double rpm) {
        io.setRPM(rpm);
    }
}
