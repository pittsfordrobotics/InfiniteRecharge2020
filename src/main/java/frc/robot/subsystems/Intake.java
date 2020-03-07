package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.Ports.*;

public class Intake extends SubsystemBase {
    private CANSparkMax m_intakeInner = new CANSparkMax(CAN.kIntakeInner, MotorType.kBrushless);
    private CANEncoder m_intakeInnerEncoder = m_intakeInner.getEncoder();

    public Intake() {
        m_intakeInner.restoreFactoryDefaults();
        m_intakeInnerEncoder.setPosition(0);
        m_intakeInner.setSmartCurrentLimit(20);
    }

    public void driveMotors(double innerSpeed) {
        m_intakeInner.setVoltage(innerSpeed);
    }
    
    @Override
    public void periodic() {
    }
}