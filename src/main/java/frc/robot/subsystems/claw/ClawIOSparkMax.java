package frc.robot.subsystems.claw;

import static frc.robot.Constants.NEO_CURRENT_LIMIT;
import static frc.robot.subsystems.claw.ClawConstants.MAX_ACCELERATION;
import static frc.robot.subsystems.claw.ClawConstants.MAX_VELOCITY;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.Logger;

public class ClawIOSparkMax implements ClawIO {
    private SparkFlex motor;
    private RelativeEncoder encoder;

    private ProfiledPIDController pidController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(MAX_VELOCITY,MAX_ACCELERATION));

    public ClawIOSparkMax() {
        motor = new SparkFlex(ClawConstants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

        encoder = motor.getEncoder();

        SparkFlexConfig config = new SparkFlexConfig();

        config
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .voltageCompensation(12)
                .smartCurrentLimit(NEO_CURRENT_LIMIT);

        SparkFlexConfig backConfig = new SparkFlexConfig();

        motor.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        inputs.velocity = getVelocity();
        inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.motorCurrent = motor.getOutputCurrent();
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void setPIDGains(double Kp, double Ki, double Kd) {
        pidController.setPID(Kp, Ki, Kd);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
        Logger.recordOutput(
                "Shooter/Set Voltage",
                voltage);
    }

    @Override
    public double getVoltage() {
        return motor.getAppliedOutput()
                * motor
                .getBusVoltage();
    }

    @Override
    public void setVelocity(double rpm) {
        double voltage = pidController.calculate(getVelocity(), rpm); 

        motor.setVoltage(voltage);
    }
}