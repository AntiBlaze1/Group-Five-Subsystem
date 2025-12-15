package frc.robot.subsystems.claw;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.claw.ClawConstants.VOLTAGE_RANGE;

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

  public Command runVoltage(DoubleSupplier voltage) {
    return run(() -> io.setVoltage(MathUtil.clamp(voltage.getAsDouble(),-VOLTAGE_RANGE,VOLTAGE_RANGE)))
            .withName("Claw Voltage");
  }

  public Command runVelocity(DoubleSupplier rpm) {
    return run(() -> io.setVelocity(rpm.getAsDouble()))
            .withName("Claw Velocity");
  }
}
