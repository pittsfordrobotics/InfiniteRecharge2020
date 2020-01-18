/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {
    private CANSparkMax m_leftPrimary = new CANSparkMax(kLeftPrimary, MotorType.kBrushless);
    private CANSparkMax m_leftFollower = new CANSparkMax(kLeftFollower, MotorType.kBrushless);

    private CANSparkMax m_rightPrimary = new CANSparkMax(kRightPrimary, MotorType.kBrushless);
    private CANSparkMax m_rightFollower = new CANSparkMax(kRightFollower, MotorType.kBrushless);

    private Joystick m_leftJoystick;
    private Joystick m_rightJoystick;

    private DifferentialDrive m_differentialDrive;

    /**
     * Creates a new DriveTrain.
     */
    public DriveTrain(Joystick leftJoystick, Joystick rightJoystick) {
        m_leftPrimary.restoreFactoryDefaults();
        m_leftFollower.restoreFactoryDefaults();
        m_leftFollower.follow(m_leftPrimary);

        m_rightPrimary.restoreFactoryDefaults();
        m_rightFollower.restoreFactoryDefaults();
        m_rightFollower.follow(m_rightPrimary);

        m_differentialDrive = new DifferentialDrive(m_leftPrimary, m_rightPrimary);
    }

    public void drive() {
        m_differentialDrive.tankDrive(-m_leftJoystick.getY() * 0.5, -m_rightJoystick.getY() * 0.5);
    }
}
