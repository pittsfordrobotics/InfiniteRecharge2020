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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.drivetrain.DriveWithXboxController;
import frc.robot.commands.intake.DriveIntake;
import frc.robot.commands.intake.ToggleIntakeExtend;
import frc.robot.commands.shooter.DriveAgitator;
import frc.robot.commands.shooter.DriveShooter;
import frc.robot.commands.shooter.MakeRoomInHopper;
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
        SmartDashboard.putData("Shooter", m_shooter);
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
        JoystickButton shiftButton = new JoystickButton(m_controller, XboxController.Button.kBumperLeft.value);

        // Drivetrain
        new POVButton(m_controller, 0).whenActive(()-> m_driveTrain.setThrottle(0.9));
        new POVButton(m_controller, 270).whenActive(()-> m_driveTrain.setThrottle(0.6));
        new POVButton(m_controller, 180).whenActive(()-> m_driveTrain.setThrottle(0.3));

        // Climber
        JoystickButton winchUpButton = new JoystickButton(m_controller, XboxController.Button.kStart.value);

        //telescopeUpButton.and(shiftButton.negate()).whenActive(new RaiseTelescopingArm(m_climber));
        //telescopeUpButton.and(shiftButton).whenActive(new LowerTelescopingArm(m_climber));

        winchUpButton.whileHeld(new WinchUp(m_climber));

        // Intake
        JoystickButton toggleIntakeExtendButton = new JoystickButton(m_controller, XboxController.Button.kY.value);
        JoystickButton driveIntakeButton = new JoystickButton(m_controller, XboxController.Button.kBumperRight.value);

        toggleIntakeExtendButton.whenPressed(new ToggleIntakeExtend(m_intake));
        driveIntakeButton.and(shiftButton.negate()).whileActiveContinuous(new DriveIntake(m_intake, false));
        driveIntakeButton.and(shiftButton).whileActiveContinuous(new DriveIntake(m_intake, true));

        // Shooter
        JoystickButton driveShooterButton = new JoystickButton(m_controller, XboxController.Button.kA.value);
        JoystickButton driveAgitatorButton = new JoystickButton(m_controller, XboxController.Button.kB.value);

        driveShooterButton.whileHeld(new DriveShooter(m_shooter));
        driveAgitatorButton.whileHeld(new DriveAgitator(m_shooter));
        driveAgitatorButton.and(shiftButton).whileActiveContinuous(new MakeRoomInHopper(m_shooter));
        
        // Spinner
        JoystickButton driveSpinnerButton = new JoystickButton(m_controller, XboxController.Button.kBack.value);

        driveSpinnerButton.whileHeld(new DriveSpinner(m_spinner));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new FollowPath(m_driveTrain, Trajectories.simpleForward);
    }
}