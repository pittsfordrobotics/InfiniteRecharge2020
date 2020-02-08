/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.Drive;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private ScaledJoystick m_joystick = new ScaledJoystick(0);
    private AHRS m_ahrs = new AHRS(Port.kMXP);

    private DriveTrain m_driveTrain = new DriveTrain(m_ahrs);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        m_ahrs.enableBoardlevelYawReset(true);
        m_driveTrain.setDefaultCommand(new DriveWithJoysticks(m_driveTrain, m_joystick));
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        var voltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA),
            Drive.kKinematics, 
            10);

        TrajectoryConfig config = new TrajectoryConfig(Drive.kMaxVelocityMetersPerSecond, Drive.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(Drive.kKinematics)
        .addConstraint(voltageConstraint);

        // Test trajectory
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(),
            new Pose2d(3, 0, new Rotation2d(0)), 
            config);

        m_driveTrain.resetOdometry(traj.getInitialPose());

        RamseteCommand ramsete = new RamseteCommand(
            traj, 
            m_driveTrain::getPose, 
            new RamseteController(), 
            new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA), 
            Drive.kKinematics, 
            m_driveTrain::getWheelSpeeds, 
            m_driveTrain.getLeftController(), 
            m_driveTrain.getRightController(), 
            m_driveTrain::driveVolts, 
            m_driveTrain);

        return ramsete.andThen(() -> m_driveTrain.driveVolts(0, 0), m_driveTrain);
    }
}