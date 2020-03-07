/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Constants.Auto;

/**
 * Class containing various trajectories for autonomous period
 */
public final class Trajectories {
    public static final Trajectory circleRight = TrajectoryGenerator.generateTrajectory(
        List.of(
           new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
           new Pose2d(1, -1, Rotation2d.fromDegrees(-90)),
           new Pose2d(0, -2, Rotation2d.fromDegrees(-180)),
           new Pose2d(-1, -1, Rotation2d.fromDegrees(-270)),
           new Pose2d(0, 0, Rotation2d.fromDegrees(0))
        ), 
        Auto.kConfig
    );

    public static final Trajectory simpleForward = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(1, 0, Rotation2d.fromDegrees(0))
        ),
        Auto.kConfig
    );

    public static final Trajectory nineInchesBack = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
            // nine inches = 0.2886 meters.
            new Pose2d(-0.2286, 0, Rotation2d.fromDegrees(180))
        ),
        Auto.kConfig
    );
}
