package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.Ports.*;

public class Intake extends SubsystemBase {
    private CANSparkMax m_intakeInner = new CANSparkMax(CAN.kIntakeInner, MotorType.kBrushless);
    private CANEncoder m_intakeInnerEncoder = m_intakeInner.getEncoder();

    private CANSparkMax m_intakeOuter = new CANSparkMax(CAN.kIntakeOuter, MotorType.kBrushless);
    private CANEncoder m_intakeOuterEncoder = m_intakeOuter.getEncoder();

    public Intake() {
        m_intakeInner.restoreFactoryDefaults();
        m_intakeInnerEncoder.setPosition(0);

        m_intakeOuter.restoreFactoryDefaults();
        m_intakeOuterEncoder.setPosition(0);
    }

    public void driveMotors(double innerSpeed, double outerSpeed) {
        m_intakeInner.set(innerSpeed);
        m_intakeOuter.set(outerSpeed);
    }
    
    @Override
    public void periodic() {
    }
}