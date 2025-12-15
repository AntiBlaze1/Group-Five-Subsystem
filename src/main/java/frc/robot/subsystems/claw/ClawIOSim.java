package frc.robot.subsystems.claw;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.claw.ClawConstants.MAX_ACCELERATION;
import static frc.robot.subsystems.claw.ClawConstants.MAX_VELOCITY;

/*
Alright I dont know what I did here but this wont run i just copied iosparkmax and some old code from FRC 1257
 */

public class ClawIOSim implements ClawIO {
    private SparkFlex motor;
    private RelativeEncoder encoder;

    private ProfiledPIDController pidController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(MAX_VELOCITY,MAX_ACCELERATION));

    public ClawIOSim() {
        motor=new SparkFlex(ClawConstants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        encoder=motor.getEncoder();
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        inputs.velocity=motor.get();
        inputs.appliedVoltage=motor.get()*12;
        inputs.motorCurrent=0.0;
    }

    /*
    Implementation of getVelocity()
     */
    @Override
    public double getVelocity() {
        return encoder.get();
    }

    /*
    Implementation of setPIDGains()
     */
    @Override
    public void setPIDGains(double Kp, double Ki, double Kd) {
        pidController.setPID(Kp, Ki, Kd);
    }

    /*
    Implementation of setVoltage()
     */
    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
        Logger.recordOutput(
                "Claw/Set Voltage",
                voltage);
    }

    /*
    Implementation of getVoltage()
     */
    @Override
    public double getVoltage() {
        return motor.getAppliedOutput()
                * motor
                .getBusVoltage();
    }

    /*
    Implementation of setVelocity()
     */
    @Override
    public void setVelocity(double rpm) {
        double voltage = pidController.calculate(getVelocity(), rpm);

        motor.setVoltage(voltage);
    }
}
