/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.Intake.*;

public class DriveIntake extends CommandBase {
    private Intake m_intake;
    private boolean m_isInverted;

    /**
     * Creates a new DriveIntake.
     */
    public DriveIntake(Intake intake, boolean isInverted)  {
        addRequirements(intake);
        m_intake = intake;
        m_isInverted = isInverted;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intake.driveMotors(m_isInverted ? -kInnerSpeed * 1.2 : kInnerSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.driveMotors(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
