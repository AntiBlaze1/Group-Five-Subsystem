package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * Real shooter IO that uses two Spark MAX motors.
 * 
 * Basically, this is the "hardware" version that talks to the actual robot.
 * It handles voltages, PID tuning, and feedforward stuff.
 */
public class ShooterIOSparkMax implements ShooterIO {

    // === Motors & encoders ===
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    // === Controllers ===
    private final PIDController leftPID;
    private final PIDController rightPID;

    private SimpleMotorFeedforward leftFF;
    private SimpleMotorFeedforward rightFF;

    // === Constructor ===
    public ShooterIOSparkMax(int leftID, int rightID) {
        // create both motors
        leftMotor = new CANSparkMax(leftID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(rightID, MotorType.kBrushless);

        // reset to known state
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        // encoders for measuring speed (in RPM)
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        // simple starting PID and FF
        leftPID = new PIDController(0.0005, 0, 0);
        rightPID = new PIDController(0.0005, 0, 0);

        leftFF = new SimpleMotorFeedforward(0, 0, 0);
        rightFF = new SimpleMotorFeedforward(0, 0, 0);

        // make sure motors spin same way
        rightMotor.follow(leftMotor, true);
    }

    // === Methods from ShooterIO ===

    /** Open-loop control: directly set voltage to both motors. */
    @Override
    public void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    /**
     * Closed-loop control: set target shooter speed (in RPM).
     * The motors will try to match that speed using PID + feedforward.
     */
    @Override
    public void setRPM(double rpm) {
        double leftSpeed = leftEncoder.getVelocity();
        double rightSpeed = rightEncoder.getVelocity();

        // basic PID correction
        double leftPIDOut = leftPID.calculate(leftSpeed, rpm);
        double rightPIDOut = rightPID.calculate(rightSpeed, rpm);

        // feedforward estimate
        double leftFFOut = leftFF.calculate(rpm);
        double rightFFOut = rightFF.calculate(rpm);

        // combine and apply
        double leftVoltage = leftPIDOut + leftFFOut;
        double rightVoltage = rightPIDOut + rightFFOut;

        leftMotor.setVoltage(leftVoltage);
        rightMotor.setVoltage(rightVoltage);
    }

    /** Sets the PID gains for both motors. */
    @Override
    public void setPIDGains(double Kp, double Ki, double Kd) {
        leftPID.setP(Kp);
        leftPID.setI(Ki);
        leftPID.setD(Kd);

        rightPID.setP(Kp);
        rightPID.setI(Ki);
        rightPID.setD(Kd);
    }

    /** Sets feedforward constants for both sides. */
    @Override
    public void setFeedforwardGains(double Ks, double Kv, double Ka) {
        leftFF = new SimpleMotorFeedforward(Ks, Kv, Ka);
        rightFF = new SimpleMotorFeedforward(Ks, Kv, Ka);
    }

    /** Stops both motors (used for safety or when command ends). */
    public void stopMotors() {
        leftMotor.set(0);
        rightMotor.set(0);
    }
}
