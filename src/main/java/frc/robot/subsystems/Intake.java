package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ScaledJoystick;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.Ports.*;

public class Intake extends SubsystemBase {
    private CANSparkMax m_intakeInner = new CANSparkMax(CAN.kIntakeInner, MotorType.kBrushless);
    private CANSparkMax m_intakeOuter = new CANSparkMax(CAN.kIntakeOuter, MotorType.kBrushless);

    private ScaledJoystick m_joystick;
    private JoystickButton m_intakeInnerButton;
    private JoystickButton m_intakeOuterButton;
    
    public Intake(ScaledJoystick joystick) {
        SmartDashboard.putNumber("Speed", 0.5);
        SmartDashboard.putBoolean("Reversed", false);

        m_intakeInner.restoreFactoryDefaults();
        m_intakeInner.getEncoder().setPosition(0);

        m_intakeOuter.restoreFactoryDefaults();
        m_intakeOuter.getEncoder().setPosition(0);

        m_joystick = joystick;
        m_intakeInnerButton = new JoystickButton(m_joystick, 4);
        m_intakeOuterButton = new JoystickButton(m_joystick, 5);
    }
    
    @Override
    public void periodic() {
        double speed = SmartDashboard.getNumber("Speed", 0.5);
        boolean reversed = SmartDashboard.getBoolean("Reversed", false);

        m_intakeInner.setInverted(reversed);

        if (m_intakeInnerButton.get()){
            m_intakeInner.set(speed);
        }
        
        if (m_intakeOuterButton.get()){
            m_intakeOuter.set(speed);
        }
    }
}