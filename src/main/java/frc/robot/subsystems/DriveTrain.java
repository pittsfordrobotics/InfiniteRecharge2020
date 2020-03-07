/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Ports.*;
import static frc.robot.Constants.Drive.*;

public class DriveTrain extends SubsystemBase {
    private CANSparkMax m_leftPrimary = new CANSparkMax(CAN.kDriveLeftPrimary, MotorType.kBrushless);
    private CANSparkMax m_leftFollower = new CANSparkMax(CAN.kDriveLeftFollower, MotorType.kBrushless);

    private CANEncoder m_leftEncoder = m_leftPrimary.getEncoder();

    private CANSparkMax m_rightPrimary = new CANSparkMax(CAN.kDriveRightPrimary, MotorType.kBrushless);
    private CANSparkMax m_rightFollower = new CANSparkMax(CAN.kDriveRightFollower, MotorType.kBrushless);

    private CANEncoder m_rightEncoder = m_rightPrimary.getEncoder();

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
        m_leftEncoder.setPosition(0);
        m_leftFollower.follow(m_leftPrimary);

        initController(m_rightPrimary);
        initController(m_rightFollower);
        m_rightEncoder.setPosition(0);
        m_rightFollower.follow(m_rightPrimary);

        resetEncoders();

        m_differentialDrive = new DifferentialDrive(m_leftPrimary, m_rightPrimary);
        m_differentialDrive.setDeadband(0.05);

        m_ahrs = ahrs;
        m_ahrs.reset();

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()));

        m_pose = new Pose2d(0, 0, Rotation2d.fromDegrees(getAngle()));
        m_wheelSpeeds = new DifferentialDriveWheelSpeeds(0, 0);

        m_leftEncoder.setPositionConversionFactor(Math.PI * kWheelDiameterMeters / kGearRatio);
        m_rightEncoder.setPositionConversionFactor(Math.PI * kWheelDiameterMeters / kGearRatio);
        m_leftEncoder.setVelocityConversionFactor(Math.PI * kWheelDiameterMeters / kGearRatio / 60);
        m_rightEncoder.setVelocityConversionFactor(Math.PI * kWheelDiameterMeters / kGearRatio / 60);

        setThrottle(0.6);
    }

    public void drive(double speed, double rotation) {
        m_differentialDrive.arcadeDrive(speed, rotation);
    }

    public void setThrottle(double throttle) {
        m_differentialDrive.setMaxOutput(throttle);
    }

    public void driveVolts(double left, double right) {
        m_leftPrimary.setVoltage(left);
        m_rightPrimary.setVoltage(-right);

        m_differentialDrive.feed();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getAngle()));
        resetEncoders();
    }

    public void resetEncoders() {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        // Distance
        double leftMeters = m_leftEncoder.getPosition(); 
        double rightMeters = -m_rightEncoder.getPosition();

        m_pose = m_odometry.update(
            Rotation2d.fromDegrees(getAngle()), 
            leftMeters,
            rightMeters);

        // Velocity
        double leftVelocity = m_leftEncoder.getVelocity();
        double rightVelocity = -m_rightEncoder.getVelocity();

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
    }
}
