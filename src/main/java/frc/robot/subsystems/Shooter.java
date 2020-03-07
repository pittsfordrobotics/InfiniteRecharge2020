/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.Ports.*;
import static frc.robot.Constants.Shooter.*;

public class Shooter extends SubsystemBase {
    private CANSparkMax m_shooterMain = new CANSparkMax(CAN.kShooterMain, MotorType.kBrushless);
    private CANEncoder m_shooterMainEncoder = m_shooterMain.getEncoder();
    private CANPIDController m_shooterMainPIDController = m_shooterMain.getPIDController();

    private CANSparkMax m_shooterFeeder = new CANSparkMax(CAN.kShooterFeeder, MotorType.kBrushless);
    private CANEncoder m_shooterFeederEncoder = m_shooterFeeder.getEncoder();

    private CANSparkMax m_shooterAgitator = new CANSparkMax(CAN.kShooterAgitator, MotorType.kBrushless);
    private CANEncoder m_shooterAgitatorEncoder = m_shooterAgitator.getEncoder();

    /**
     * Creates a new Shooter.
     */
    public Shooter() {
        m_shooterMain.restoreFactoryDefaults();
        m_shooterMainEncoder.setPosition(0);
        m_shooterMain.setInverted(true);
        m_shooterMain.setClosedLoopRampRate(2);

        m_shooterFeeder.restoreFactoryDefaults();
        m_shooterFeederEncoder.setPosition(0);
        m_shooterFeeder.setInverted(true);
        m_shooterFeeder.setSmartCurrentLimit(20);

        m_shooterAgitator.restoreFactoryDefaults();
        m_shooterAgitatorEncoder.setPosition(0);
        m_shooterAgitator.setSmartCurrentLimit(20);
    }

    public boolean isUpToSpeed() {
        return m_shooterMain.getEncoder().getVelocity() > kSpeedThreshold;
    }

    public void driveMain(double speed) {
        if (m_shooterMain.getEncoder().getVelocity() >= speed) {
            m_shooterMainPIDController.setP(0);
        } else {
            m_shooterMainPIDController.setP(kP);
        }

        m_shooterMainPIDController.setFF(kF);
        m_shooterMainPIDController.setReference(speed, ControlType.kVelocity);
    }

    public void driveFeeder(double speed) {
        m_shooterFeeder.setVoltage(speed);
    }

    public void driveAgitator(double speed) {
        m_shooterAgitator.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("Shooter", this);
        SmartDashboard.putNumber("Shooter Velocity", m_shooterMainEncoder.getVelocity());
        SmartDashboard.putNumber("Feeder Velocity", m_shooterFeederEncoder.getVelocity());
    }
}
