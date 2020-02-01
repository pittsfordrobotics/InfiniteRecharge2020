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

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Ports.*;
import static frc.robot.Constants.Drive.*;

public class DriveTrain extends SubsystemBase {
    private CANSparkMax m_leftPrimary = new CANSparkMax(CAN.kLeftPrimary, MotorType.kBrushless);
    private CANSparkMax m_leftFollower = new CANSparkMax(CAN.kLeftFollower, MotorType.kBrushless);

    private CANSparkMax m_rightPrimary = new CANSparkMax(CAN.kRightPrimary, MotorType.kBrushless);
    private CANSparkMax m_rightFollower = new CANSparkMax(CAN.kRightFollower, MotorType.kBrushless);

    private DifferentialDrive m_differentialDrive;
    private DifferentialDriveOdometry m_odometry;
    private DifferentialDriveWheelSpeeds m_wheelSpeeds = new DifferentialDriveWheelSpeeds();
    private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kWheelDiameterMeters);
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
        m_ahrs.reset();

        m_pose = new Pose2d(0, 0, new Rotation2d(m_ahrs.getAngle()));
    }

    public void drive(double speed, double rotation) {
        m_differentialDrive.arcadeDrive(speed, rotation);

        SmartDashboard.putNumber("Right Primary", m_rightPrimary.get());
        SmartDashboard.putNumber("Right Follower", m_rightFollower.get());
        SmartDashboard.putNumber("Left Primary", m_leftPrimary.get());
        SmartDashboard.putNumber("Left Follower", m_leftFollower.get());

        SmartDashboard.putNumber("Left ticks", m_leftPrimary.getEncoder().getPosition());
        SmartDashboard.putNumber("Ticks per rotation", m_leftPrimary.getEncoder().getCountsPerRevolution());
        SmartDashboard.putNumber("Conversion factor", m_leftPrimary.getEncoder().getPositionConversionFactor());
    }

    public void driveVolts(double left, double right) {
        m_differentialDrive.tankDrive(left, right);

        double leftMeters = getControllerEncoderDistanceMeters(m_leftPrimary, false);
        double rightMeters = getControllerEncoderDistanceMeters(m_rightPrimary, true);

        SmartDashboard.putNumber("Left Meters", leftMeters);
        SmartDashboard.putNumber("Right Meters", rightMeters);

        m_pose = m_odometry.update(
            new Rotation2d(m_ahrs.getAngle()), 
            leftMeters,
            rightMeters);
        
        m_leftPrimary.setVoltage(left);
        m_rightPrimary.setVoltage(right);

        double leftVelocity = getControllerEncoderVelocityMetersPerSecond(m_leftPrimary);
        double rightVelocity = getControllerEncoderVelocityMetersPerSecond(m_rightPrimary);

        m_wheelSpeeds = new DifferentialDriveWheelSpeeds(leftVelocity, rightVelocity);
        
        SmartDashboard.putNumber("Right Primary", m_rightPrimary.get());
        SmartDashboard.putNumber("Right Follower", m_rightFollower.get());
        SmartDashboard.putNumber("Left Primary", m_leftPrimary.get());
        SmartDashboard.putNumber("Left Follower", m_leftFollower.get());
    }

    public PIDController getLeftController() {
        return new PIDController(kP, kI, kD);
    }

    public PIDController getRightController() {
        return new PIDController(kP, kI, kD);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return m_wheelSpeeds;
    }

    public Pose2d getPose() {
        return m_pose;
    }

    public DifferentialDriveKinematics getKinematics() {
        return m_kinematics;
    }

    private double getControllerEncoderDistanceMeters(CANSparkMax controller, boolean isInverted) {
        double numMotorRotations = controller.getEncoder().getPosition();
        double numWheelRotations = numMotorRotations / kGearRatio;
        double wheelCircumference = Math.PI * kWheelDiameterMeters;
        double distance = numWheelRotations * wheelCircumference;

        return isInverted ? -distance : distance;
    }

    private double getControllerEncoderVelocityMetersPerSecond(CANSparkMax controller) {
        double motorVelocity = controller.getEncoder().getVelocity() / 60;
        double wheelVelocity = motorVelocity / kGearRatio;
        double wheelCircumference = Math.PI * kWheelDiameterMeters;
        double velocity = wheelVelocity * wheelCircumference;

        return velocity;
    }

    private void initController(CANSparkMax controller) {
        controller.restoreFactoryDefaults();
        controller.setIdleMode(IdleMode.kBrake);
        controller.getEncoder().setPosition(0);
    }
}
