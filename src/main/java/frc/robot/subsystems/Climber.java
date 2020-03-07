/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports.*;
import static frc.robot.Constants.Climber.*;

public class Climber extends SubsystemBase {
    private CANSparkMax m_telescopingArm = new CANSparkMax(CAN.kClimberTelescopingArm, MotorType.kBrushless);
    private CANEncoder m_telescopingArmEncoder = m_telescopingArm.getEncoder();

    private CANSparkMax m_winch = new CANSparkMax(CAN.kClimberWinch, MotorType.kBrushless);
    private CANEncoder m_winchEncoder = m_winch.getEncoder();

    /**
    * Creates a new Climber.
    */
    public Climber() {
        m_telescopingArm.restoreFactoryDefaults();
        m_telescopingArmEncoder.setPosition(0);
        m_telescopingArm.setIdleMode(IdleMode.kBrake);
        m_telescopingArm.setSmartCurrentLimit(20);

        m_winch.restoreFactoryDefaults();
        m_winchEncoder.setPosition(0);
    }

    public void driveTelescopingArm(double speed) {
        m_telescopingArm.set(speed);
    }

    public void driveWinch(double speed) {
        m_winch.set(speed);
    }

    public boolean isTelescopingArmExtended() {
        return m_telescopingArmEncoder.getPosition() >= kMaxTelescopingArmPosition;
    }

    public boolean isTelescopingArmRetracted() {
        return m_telescopingArmEncoder.getPosition() <= 0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Telescope Position", m_telescopingArmEncoder.getPosition());
    }
}