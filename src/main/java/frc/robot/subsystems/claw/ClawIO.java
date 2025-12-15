package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;


/*
Interface for the Claw subsystem
 */
public interface ClawIO {
  @AutoLog
  public static class ClawIOInputs {
    public double velocity=0.0; //rpm
    public double appliedVoltage=0.0;
    public double motorCurrent=0.0;
  }

  public default void updateInputs(ClawIOInputs inputs) {}

  /*
  Get the velocity (RPM)
   */
  public default double getVelocity() {
    return 0.0;
  }

  /*
  Get the voltage
   */
  public default double getVoltage() {
    return 0.0;
  }

  /* Set the velocity */
  public default void setVelocity(double rpm) {}

  /* Set the voltage */
  public default void setVoltage(double voltage) {}

  /* Set the PID of the subsystem */
  public default void setPIDGains(double Kp, double Ki, double Kd) {}
}
