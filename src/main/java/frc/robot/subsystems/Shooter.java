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
    private CANSparkMax m_shooterMain = new CANSparkMax(CAN.kShooterMain, MotorType.kBrushless);
    private CANSparkMax m_shooterFeed = new CANSparkMax(CAN.kShooterFeed, MotorType.kBrushless);

    private ScaledJoystick m_joystick;
    /**
     * Creates a new Shooter.
     */
    public Shooter(ScaledJoystick joystick) {
        m_joystick = joystick;

        SmartDashboard.putNumber("Max Speed Main", 0.5);
        SmartDashboard.putBoolean("Invert Motor Main", true);

        SmartDashboard.putNumber("Max Speed Feed", 0.5);
        SmartDashboard.putBoolean("Invert Motor Feed", true);

        m_shooterMain.restoreFactoryDefaults();
        m_shooterMain.getEncoder().setPosition(0);

        m_shooterFeed.restoreFactoryDefaults();
        m_shooterFeed.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double maxSpeedMain = SmartDashboard.getNumber("Max Speed Main", 0.5);
        boolean invertMain = SmartDashboard.getBoolean("Invert Motor Main", true);

        double maxSpeedFeed = SmartDashboard.getNumber("Max Speed Main", 0.5);
        boolean invertFeed = SmartDashboard.getBoolean("Invert Motor Main", true);

        m_shooterMain.setInverted(invertMain);
        m_shooterFeed.setInverted(invertFeed);

        m_shooterMain.set(m_joystick.getScaledThrottle() * maxSpeedMain);
        m_shooterFeed.set(m_joystick.getScaledThrottle() * maxSpeedFeed);
    }
}
