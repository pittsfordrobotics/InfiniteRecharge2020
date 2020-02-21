/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.Ports.*;

public class Shooter extends SubsystemBase {
    private CANSparkMax m_shooterMain = new CANSparkMax(CAN.kShooterMain, MotorType.kBrushless);
    private CANSparkMax m_shooterFeeder = new CANSparkMax(CAN.kShooterFeeder, MotorType.kBrushless);
    private CANSparkMax m_shooterAgitator = new CANSparkMax(CAN.kShooterAgitator, MotorType.kBrushless);

    /**
     * Creates a new Shooter.
     */
    public Shooter() {
        SmartDashboard.putNumber("Speed Main", 0.5);
        SmartDashboard.putNumber("Speed Feeder", 0.5);
        SmartDashboard.putNumber("Speed Agitator", 0.5);

        m_shooterMain.restoreFactoryDefaults();
        m_shooterMain.getEncoder().setPosition(0);
        m_shooterMain.setInverted(true);

        m_shooterFeeder.restoreFactoryDefaults();
        m_shooterFeeder.getEncoder().setPosition(0);
        m_shooterFeeder.follow(m_shooterMain);

        m_shooterAgitator.restoreFactoryDefaults();
        m_shooterAgitator.getEncoder().setPosition(0);
    }

    public void driveMain(double speed) {
        m_shooterMain.set(speed);
    }

    public void driveFeeder(double speed) {
        m_shooterFeeder.set(speed);
    }

    public void driveAgitator(double speed) {
        m_shooterAgitator.set(speed);
    }

    @Override
    public void periodic() {
    }
}
