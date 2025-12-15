package frc.robot.subsystems.claw;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.claw.ClawConstants.VOLTAGE_RANGE;


/*
The actual subsystem, includes everything to be exposed to the outside.
 */
public class Claw extends SubsystemBase {
  private ClawIO io;
  private ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

  public Claw(ClawIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Claw",inputs);
  }

  /*
  Sets the voltage to run the motor at.
  */
  public Command runVoltage(DoubleSupplier voltage) {
    return run(() -> io.setVoltage(MathUtil.clamp(voltage.getAsDouble(),-VOLTAGE_RANGE,VOLTAGE_RANGE)))
            .withName("Claw Voltage");
  }

  /*
  Sets the velocity to run the motor at.
   */
  public Command runVelocity(DoubleSupplier rpm) {
    return run(() -> io.setVelocity(rpm.getAsDouble()))
            .withName("Claw Velocity");
  }
}
