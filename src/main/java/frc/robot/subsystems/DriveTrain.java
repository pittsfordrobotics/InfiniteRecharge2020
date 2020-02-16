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
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Ports.*;
import static frc.robot.Constants.Drive.*;

public class DriveTrain extends SubsystemBase {
    private CANSparkMax m_leftPrimary = new CANSparkMax(CAN.kDriveLeftPrimary, MotorType.kBrushless);
    private CANSparkMax m_leftFollower = new CANSparkMax(CAN.kDriveLeftFollower, MotorType.kBrushless);

    private CANSparkMax m_rightPrimary = new CANSparkMax(CAN.kDriveRightPrimary, MotorType.kBrushless);
    private CANSparkMax m_rightFollower = new CANSparkMax(CAN.kDriveRightFollower, MotorType.kBrushless);

    private DifferentialDrive m_differentialDrive;
    private DifferentialDriveOdometry m_odometry;
    private DifferentialDriveWheelSpeeds m_wheelSpeeds;
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

        resetEncoders();

        m_differentialDrive = new DifferentialDrive(m_leftPrimary, m_rightPrimary);

        m_ahrs = ahrs;
        m_ahrs.reset();

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()));

        m_pose = new Pose2d(0, 0, Rotation2d.fromDegrees(getAngle()));
        m_wheelSpeeds = new DifferentialDriveWheelSpeeds(0, 0);

        m_leftPrimary.getEncoder().setPositionConversionFactor(Math.PI * kWheelDiameterMeters / kGearRatio);
        m_rightPrimary.getEncoder().setPositionConversionFactor(Math.PI * kWheelDiameterMeters / kGearRatio);
        m_leftPrimary.getEncoder().setVelocityConversionFactor(Math.PI * kWheelDiameterMeters / kGearRatio / 60);
        m_rightPrimary.getEncoder().setVelocityConversionFactor(Math.PI * kWheelDiameterMeters / kGearRatio / 60);
    }

    public void drive(double speed, double rotation) {
        m_differentialDrive.arcadeDrive(speed, rotation);

        SmartDashboard.putNumber("Right Primary", m_rightPrimary.get());
        SmartDashboard.putNumber("Right Follower", m_rightFollower.get());
        SmartDashboard.putNumber("Left Primary", m_leftPrimary.get());
        SmartDashboard.putNumber("Left Follower", m_leftFollower.get());
    }

    public void setThrottle(double throttle) {
        m_differentialDrive.setMaxOutput(throttle);
    }

    public void driveVolts(double left, double right) {
        m_leftPrimary.setVoltage(left);
        m_rightPrimary.setVoltage(-right);

        SmartDashboard.putNumber("Right Set Voltage", right);
        SmartDashboard.putNumber("Left Set Voltage", left);

        m_differentialDrive.feed();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getAngle()));
        resetEncoders();
    }

    public void resetEncoders() {
        m_leftPrimary.getEncoder().setPosition(0);
        m_rightPrimary.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        // Distance
        double leftMeters = m_leftPrimary.getEncoder().getPosition(); 
        double rightMeters = -m_rightPrimary.getEncoder().getPosition();

        SmartDashboard.putNumber("Left Meters", leftMeters);
        SmartDashboard.putNumber("Right Meters", rightMeters);
        SmartDashboard.putNumber("gyro angle", m_ahrs.getAngle());
        SmartDashboard.putNumber("Pose angle", m_pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Pose X", m_pose.getTranslation().getX());
        SmartDashboard.putNumber("Pose Y", m_pose.getTranslation().getY());

        m_pose = m_odometry.update(
            Rotation2d.fromDegrees(getAngle()), 
            leftMeters,
            rightMeters);

        // Velocity
        double leftVelocity = m_leftPrimary.getEncoder().getVelocity();
        double rightVelocity = -m_rightPrimary.getEncoder().getVelocity();

        SmartDashboard.putNumber("Left Velocity", leftVelocity);
        SmartDashboard.putNumber("Right Velocity", rightVelocity);

        m_wheelSpeeds = new DifferentialDriveWheelSpeeds(leftVelocity, rightVelocity);
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

    private double getAngle() {
        return -m_ahrs.getAngle();
    }

    private void initController(CANSparkMax controller) {
        controller.restoreFactoryDefaults();
        controller.setIdleMode(IdleMode.kBrake);
        controller.getEncoder().setPosition(0);
    }
}
