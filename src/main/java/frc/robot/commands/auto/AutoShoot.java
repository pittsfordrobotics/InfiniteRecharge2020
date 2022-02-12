/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.EnhancedShooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
    private Shooter m_shooter;
    private Intake m_intake;
    private DriveTrain m_driveTrain;

    /**
     * Creates a new AutoShoot.
     */
    public AutoShoot(Shooter shooter, Intake intake, DriveTrain driveTrain, Trajectory traj) {
        super(
            new FollowPath(driveTrain, traj),
            new EnhancedShooter(shooter, intake)
        );

        m_shooter = shooter;
        m_intake = intake;
        m_driveTrain = driveTrain;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.driveAgitator(0);
        m_shooter.driveFeeder(0);
        m_shooter.driveMain(0);

        m_intake.driveMotors(0);
        
        m_driveTrain.driveVolts(0, 0);
    }
}
