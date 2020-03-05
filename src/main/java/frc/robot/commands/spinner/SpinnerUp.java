/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.spinner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spinner;

public class SpinnerUp extends CommandBase {
    private Spinner m_spinner;

    /**
     * Creates a new ToggleSpinnerUpDown.
     */
    public SpinnerUp(Spinner spinner) {
        addRequirements(spinner);
        m_spinner = spinner;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled. 
    @Override 
    public void execute() {
        m_spinner.raise();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_spinner.stopUpDown();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_spinner.isRaised();
    }
}
