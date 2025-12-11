package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
  @AutoLog
  public static class ClawIOInputs {
    public double velocity=0.0; //rpm
    public double appliedVoltage=0.0;
    public double motorCurrent=0.0;
  }

  public default void updateInputs(ClawIOInputs inputs) {}

  public default double getVelocity() {
    return 0.0;
  }

  public default double getVoltage() {
    return 0.0;
  }

  public default void setVelocity(double rpm) {}

  public default void setVoltage(double voltage) {}

  public default void setPIDGains(double Kp, double Ki, double Kd) {}
}
