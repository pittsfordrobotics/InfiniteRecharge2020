/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveTrain;

public class DriveWithXboxController extends CommandBase {
    private DriveTrain m_driveTrain;
    private XboxController m_controller;
    private double throttle;
    private double limitedThrottle;
    private Timer m_timer = new Timer();

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
        m_timer.reset();
        m_timer.start();
        throttle = 0;
        limitedThrottle = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_timer.get() >= 0.2) {
            throttle = -m_controller.getY(Hand.kLeft);
            limitedThrottle += Math.abs(throttle - limitedThrottle) >= 0.2 ? 0.2 * Math.signum(throttle) : throttle;
            limitedThrottle = Math.abs(limitedThrottle) > 1 ? 1 * Math.signum(limitedThrottle) : limitedThrottle;
            m_timer.reset();
        }
        m_driveTrain.drive(limitedThrottle, m_controller.getX(Hand.kRight) * 0.75);

        System.out.println("Left Velocity:" + m_driveTrain.getLeftVelocity());
        System.out.println("Right Velocity:" + m_driveTrain.getRightVelocity());
    }    

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
