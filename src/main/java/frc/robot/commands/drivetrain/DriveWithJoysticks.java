/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ScaledJoystick;
import frc.robot.subsystems.DriveTrain;

public class DriveWithJoysticks extends CommandBase {
    private DriveTrain m_driveTrain;
    private ScaledJoystick m_joystick;

    /**
     * Creates a new DriveWithJoysticks.
     */
    public DriveWithJoysticks(DriveTrain driveTrain, ScaledJoystick joystick) {
        addRequirements(driveTrain);
        m_driveTrain = driveTrain;
        m_joystick = joystick;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_driveTrain.drive(m_joystick.getScaledY(), m_joystick.getScaledTwist());
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
