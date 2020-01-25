/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {
    private static final double kGearRatioMultiplier = 10;

    private CANSparkMax m_leftPrimary = new CANSparkMax(Ports.kLeftPrimary, MotorType.kBrushless);
    private CANSparkMax m_leftFollower = new CANSparkMax(Ports.kLeftFollower, MotorType.kBrushless);

    private CANSparkMax m_rightPrimary = new CANSparkMax(Ports.kRightPrimary, MotorType.kBrushless);
    private CANSparkMax m_rightFollower = new CANSparkMax(Ports.kRightFollower, MotorType.kBrushless);

    private DifferentialDrive m_differentialDrive;
    private DifferentialDriveOdometry m_odometry;
    private Pose2d m_pose;
    private AHRS m_ahrs;

    /**
     * Creates a new DriveTrain.
     */
    public DriveTrain(AHRS ahrs) {
        initController(m_leftPrimary);
        initController(m_leftFollower);
        m_leftFollower.follow(m_leftPrimary);

        initController(m_rightPrimary);
        initController(m_rightFollower);
        m_rightFollower.follow(m_rightPrimary);

        m_differentialDrive = new DifferentialDrive(m_leftPrimary, m_rightPrimary);

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(ahrs.getAngle()));

        m_ahrs = ahrs;

        m_pose = new Pose2d(0, 0, new Rotation2d(0));
    }

    public void drive(double speed, double rotation) {
        m_differentialDrive.arcadeDrive(speed, rotation);

        SmartDashboard.putNumber("Right Primary", m_rightPrimary.get());
        SmartDashboard.putNumber("Right Follower", m_rightFollower.get());
        SmartDashboard.putNumber("Left Primary", m_leftPrimary.get());
        SmartDashboard.putNumber("Left Follower", m_leftFollower.get());

        SmartDashboard.putNumber("Left ticks", m_leftPrimary.getEncoder().getPosition());
    }

    public void tankDrive(double left, double right) {
        m_differentialDrive.tankDrive(left, right);

        double leftMeters = getControllerEncoderDistanceMeters(m_leftPrimary);
        double rightMeters = getControllerEncoderDistanceMeters(m_rightPrimary);

        SmartDashboard.putNumber("Left Meters", leftMeters);
        SmartDashboard.putNumber("Right Meters", rightMeters);

        m_pose = m_odometry.update(
            new Rotation2d(0), 
            getControllerEncoderDistanceMeters(m_leftPrimary),
            getControllerEncoderDistanceMeters(m_rightPrimary));

        SmartDashboard.putNumber("Right Primary", m_rightPrimary.get());
        SmartDashboard.putNumber("Right Follower", m_rightFollower.get());
        SmartDashboard.putNumber("Left Primary", m_leftPrimary.get());
        SmartDashboard.putNumber("Left Follower", m_leftFollower.get());
    }

    public Pose2d getPose() {
        return m_pose;
    }

    private static double getControllerEncoderDistanceMeters(CANSparkMax controller) {
        return -controller.getEncoder().getPosition() / (controller.getEncoder().getCountsPerRevolution() * kGearRatioMultiplier) * Math.PI * Units.inchesToMeters(Dimensions.kWheelDiameterInches);
    }

    private static void initController(CANSparkMax controller) {
        controller.restoreFactoryDefaults();
        controller.setIdleMode(IdleMode.kBrake);
        controller.getEncoder().setPosition(0);
    }
}
