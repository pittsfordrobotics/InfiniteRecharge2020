/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports.*;

public class Climber extends SubsystemBase {
    private CANSparkMax m_telescopingArm = new CANSparkMax(CAN.kClimberTelescopingArm, MotorType.kBrushless);
    private CANSparkMax m_winch = new CANSparkMax(CAN.kClimberWinch, MotorType.kBrushless);

    /**
    * Creates a new Climber.
    */
    public Climber() {
        SmartDashboard.putNumber("Telescoping Arm Speed", 0.25);
        SmartDashboard.putNumber("Winch Speed", 0.2);

        m_telescopingArm.restoreFactoryDefaults();
        m_telescopingArm.getEncoder().setPosition(0);
        m_telescopingArm.setIdleMode(IdleMode.kBrake);

        m_winch.restoreFactoryDefaults();
        m_winch.getEncoder().setPosition(0);
    }

    public void driveTelescopingArm(double speed) {
        m_telescopingArm.set(speed);
    }

    public void driveWinch(double speed) {
        m_winch.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Telescope Encoder", m_telescopingArm.getEncoder().getPosition());
    }
}