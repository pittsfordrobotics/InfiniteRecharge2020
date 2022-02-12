/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FollowPath extends SequentialCommandGroup {
  /**
   * Creates a new FollowPath.
   */
  public FollowPath(DriveTrain driveTrain, Trajectory traj) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
        new InstantCommand(() -> driveTrain.resetOdometry(traj.getInitialPose()), driveTrain),
        new RamseteCommand(
            traj, 
            driveTrain::getPose, 
            new RamseteController(),
            new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA), 
            Drive.kKinematics, 
            driveTrain::getWheelSpeeds, 
            driveTrain.getLeftController(), 
            driveTrain.getRightController(), 
            driveTrain::driveVolts, 
            driveTrain),
        new InstantCommand(() -> driveTrain.driveVolts(0, 0), driveTrain)
    );
  }
}
