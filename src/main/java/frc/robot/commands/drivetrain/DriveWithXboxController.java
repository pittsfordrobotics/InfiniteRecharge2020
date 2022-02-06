/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveWithXboxController extends CommandBase {
    private DriveTrain m_driveTrain;
    private XboxController m_controller;
    private double pastInput;
    private boolean decelerate;

    /**
     * Creates a new DriveWithJoysticks.
     */
    public DriveWithXboxController(DriveTrain driveTrain, XboxController xboxController) {
        addRequirements(driveTrain);
        m_driveTrain = driveTrain;
        m_controller = xboxController;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        decelerate = false;
        pastInput = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pastInput = m_controller.getY(Hand.kLeft);
        if (Math.abs(applyDeadband((m_driveTrain.getLeftVelocity()+m_driveTrain.getRightVelocity())/2,0.2)) == 0) {
            decelerate = false;
        }
        else if (!decelerate) {
            decelerate = m_controller.getY(Hand.kLeft) != 0 && (m_controller.getY(Hand.kLeft) > 0 ? m_controller.getY(Hand.kLeft) - pastInput <= 0 : m_controller.getY(Hand.kLeft) - pastInput >= 0);
        }

        if (decelerate) {
            m_driveTrain.curveDrive(m_driveTrain.getRateLimit().calculate(-m_controller.getY(Hand.kLeft)), m_controller.getX(Hand.kRight) * 0.75);
        }
        else {
            m_driveTrain.curveDrive(-m_controller.getY(Hand.kLeft), m_controller.getX(Hand.kRight) * 0.75);
            m_driveTrain.getRateLimit().calculate(-m_controller.getY(Hand.kLeft));
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.driveVolts(0,0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    private static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }
}