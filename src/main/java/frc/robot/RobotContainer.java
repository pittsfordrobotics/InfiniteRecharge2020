/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.drivetrain.DriveWithXboxController;
import frc.robot.commands.intake.DriveIntake;
import frc.robot.commands.intake.ToggleIntakeExtend;
import frc.robot.commands.shooter.DriveShooter;
import frc.robot.commands.shooter.FeedShooter;
import frc.robot.commands.spinner.DriveSpinner;
import frc.robot.commands.auto.FollowPath;
import frc.robot.commands.climber.LowerTelescopingArm;
import frc.robot.commands.climber.RaiseTelescopingArm;
import frc.robot.commands.climber.WinchUp;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private XboxController m_controller = new XboxController(0);
    private ScaledJoystick m_joystick = new ScaledJoystick(1);
    private AHRS m_ahrs = new AHRS(Port.kMXP);
    private DriveTrain m_driveTrain = new DriveTrain(m_ahrs);
    private Climber m_climber = new Climber();
    private Shooter m_shooter = new Shooter();
    private Intake m_intake = new Intake();
    private Spinner m_spinner = new Spinner();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        m_driveTrain.setDefaultCommand(new DriveWithXboxController(m_driveTrain, m_controller));
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Drivetrain
        new POVButton(m_controller, 0).whenActive(()-> m_driveTrain.setThrottle(0.9));
        new POVButton(m_controller, 270).whenActive(()-> m_driveTrain.setThrottle(0.6));
        new POVButton(m_controller, 180).whenActive(()-> m_driveTrain.setThrottle(0.3));

        // Climber
        new JoystickButton(m_controller, 1).whileHeld(new RaiseTelescopingArm(m_climber));
        new JoystickButton(m_controller, 2).whileHeld(new LowerTelescopingArm(m_climber));
        new JoystickButton(m_controller, 3).whileHeld(new WinchUp(m_climber));

        // Intake
        new JoystickButton(m_controller, 4).whenPressed(new ToggleIntakeExtend(m_intake));
        new JoystickButton(m_controller, 5).whileHeld(new DriveIntake(m_intake));

        // Shooter
        new JoystickButton(m_joystick, 4).whileHeld(new DriveShooter(m_shooter));
        new JoystickButton(m_joystick, 5).whileHeld(new FeedShooter(m_shooter));
        
        // Spinner
        new JoystickButton(m_joystick, 6).whileHeld(new DriveSpinner(m_spinner));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new FollowPath(m_driveTrain, Trajectories.circleRight);
    }
}