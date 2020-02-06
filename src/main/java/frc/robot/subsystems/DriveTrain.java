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

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Ports.*;

public class DriveTrain extends SubsystemBase {
    private CANSparkMax m_leftPrimary = new CANSparkMax(CAN.kLeftPrimary, MotorType.kBrushless);
    private CANSparkMax m_leftFollower = new CANSparkMax(CAN.kLeftFollower, MotorType.kBrushless);

    private CANSparkMax m_rightPrimary = new CANSparkMax(CAN.kRightPrimary, MotorType.kBrushless);
    private CANSparkMax m_rightFollower = new CANSparkMax(CAN.kRightFollower, MotorType.kBrushless);

    private DifferentialDrive m_differentialDrive;

    /**
     * Creates a new DriveTrain.
     */
    public DriveTrain() {
        initController(m_leftPrimary);
        initController(m_leftFollower);
        m_leftFollower.follow(m_leftPrimary);

        initController(m_rightPrimary);
        initController(m_rightFollower);
        m_rightFollower.follow(m_rightPrimary);

        m_differentialDrive = new DifferentialDrive(m_leftPrimary, m_rightPrimary);
    }

    public void drive(double speed, double rotation) {
        m_differentialDrive.arcadeDrive(speed, rotation);
        SmartDashboard.putNumber("Right Primary", m_rightPrimary.get());
        SmartDashboard.putNumber("Right Follower", m_rightFollower.get());
        SmartDashboard.putNumber("Left Primary", m_leftPrimary.get());
        SmartDashboard.putNumber("Left Follower", m_leftFollower.get());
    }

    private static void initController(CANSparkMax controller) {
        controller.restoreFactoryDefaults();
        controller.setIdleMode(IdleMode.kBrake);
    }
}
