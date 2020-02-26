package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PWM;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.Ports.*;
import static frc.robot.Constants.Intake.*;

public class Intake extends SubsystemBase {
    private CANSparkMax m_intakeInner = new CANSparkMax(CAN.kIntakeInner, MotorType.kBrushless);
    private CANSparkMax m_intakeOuter = new CANSparkMax(CAN.kIntakeOuter, MotorType.kBrushless);

    private Servo m_leftActuator = new Servo(PWM.kIntakeLeft);
    private Servo m_rightActuator = new Servo(PWM.kIntakeRight);

    private boolean m_isExtended = false;

    public Intake() {
        SmartDashboard.putNumber("Speed Inner", 0.3);
        SmartDashboard.putNumber("Speed Outer", 0.5);

        SmartDashboard.putNumber("Intake Extend", 1);

        m_intakeInner.restoreFactoryDefaults();
        m_intakeInner.getEncoder().setPosition(0);

        m_intakeOuter.restoreFactoryDefaults();
        m_intakeOuter.getEncoder().setPosition(0);

        m_leftActuator.setBounds(kMaxActuatorPulse, kMaxActuatorDeadband, kCenterActuatorPulse, kMinActuatorDeadband, kMinActuatorPulse);
        m_rightActuator.setBounds(kMaxActuatorPulse, kMaxActuatorDeadband, kCenterActuatorPulse, kMinActuatorDeadband, kMinActuatorPulse);
    }

    public void extend(double setpoint) {
        m_leftActuator.set(setpoint);
        m_rightActuator.set(setpoint);
        m_isExtended = true;
    }

    public void retract() {
        m_leftActuator.set(0);
        m_rightActuator.set(0);
        m_isExtended = false;
    }

    public void driveMotors(double innerSpeed, double outerSpeed) {
        m_intakeInner.set(innerSpeed);
        m_intakeOuter.set(outerSpeed);
    }

    public boolean isExtended() {
        return m_isExtended;
    }
    
    @Override
    public void periodic() {
    }
}