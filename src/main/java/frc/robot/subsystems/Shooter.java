/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ScaledJoystick;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.Ports.*;

public class Shooter extends SubsystemBase {

    private CANSparkMax m_sparkMax = new CANSparkMax(CAN.kShooterMain, MotorType.kBrushless);
    private ScaledJoystick m_joystick;
    /**
     * Creates a new Shooter.
     */
    public Shooter(ScaledJoystick joystick) {
        m_joystick = joystick;
        SmartDashboard.putNumber("Max Speed", 0.5);
        SmartDashboard.putBoolean("Invert Motor", true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double maxSpeed = SmartDashboard.getNumber("Max Speed", 0.5);
        boolean invert = SmartDashboard.getBoolean("Invert Motor", true);
        if (invert){
            m_sparkMax.set(m_joystick.getScaledThrottle()*maxSpeed*-1);
        }
        else {
            m_sparkMax.set(m_joystick.getScaledThrottle()*maxSpeed);
        }

    }
}
