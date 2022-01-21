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
    private boolean accelerate;

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
        accelerate = false;
        pastInput = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (accelerate && Math.abs(m_driveTrain.getLeftVelocity()) > 0) {
            accelerate = true;
        }
        else {
            accelerate = -m_controller.getY(Hand.kLeft) - pastInput < 0;
        }
        pastInput = -m_controller.getY(Hand.kLeft);
        if (accelerate) {
            m_driveTrain.drive(m_driveTrain.getRateLimit().calculate(-m_controller.getY(Hand.kLeft)), m_controller.getX(Hand.kRight) * 0.75);
        }
        else {
            m_driveTrain.drive(-m_controller.getY(Hand.kLeft), m_controller.getX(Hand.kRight) * 0.75);
        }
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