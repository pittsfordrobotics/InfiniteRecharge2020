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
    private CANSparkMax m_shooterBeltFeeder = new CANSparkMax(CAN.kShooterBeltFeeder, MotorType.kBrushless);

    /**
     * Creates a new Shooter.
     */
    public Shooter() {
        SmartDashboard.putNumber("Speed Main", 0.5);
        SmartDashboard.putNumber("Speed Feeder", 0.5);
        SmartDashboard.putNumber("Speed Belt Feeder", 0.5);

        m_shooterMain.restoreFactoryDefaults();
        m_shooterMain.getEncoder().setPosition(0);

        m_shooterFeeder.restoreFactoryDefaults();
        m_shooterFeeder.getEncoder().setPosition(0);
    }

    public void driveMotors(double mainSpeed, double feederSpeed, double beltFeederSpeed) {
        m_shooterMain.set(mainSpeed);
        m_shooterFeeder.set(feederSpeed);
        m_shooterBeltFeeder.set(beltFeederSpeed);
    }

    @Override
    public void periodic() {
    }
}
