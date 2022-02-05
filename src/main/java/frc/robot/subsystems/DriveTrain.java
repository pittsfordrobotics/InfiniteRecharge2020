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
import edu.wpi.first.wpilibj.SlewRateLimiter;
import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Drive.*;
import static frc.robot.Constants.Drive.kD;
import static frc.robot.Constants.Ports.CAN;

public class DriveTrain extends SubsystemBase {
    private CANSparkMax m_leftPrimary = new CANSparkMax(CAN.kDriveLeftPrimary, MotorType.kBrushless);
    private CANSparkMax m_leftFollower = new CANSparkMax(CAN.kDriveLeftFollower, MotorType.kBrushless);

    private CANEncoder m_leftEncoder = m_leftPrimary.getEncoder();
    private CANPIDController m_leftPidController = m_leftPrimary.getPIDController();

    private CANSparkMax m_rightPrimary = new CANSparkMax(CAN.kDriveRightPrimary, MotorType.kBrushless);
    private CANSparkMax m_rightFollower = new CANSparkMax(CAN.kDriveRightFollower, MotorType.kBrushless);

    private CANEncoder m_rightEncoder = m_rightPrimary.getEncoder();
    private CANPIDController m_rightPidController = m_rightPrimary.getPIDController();

    private DifferentialDrive m_differentialDrive;
    private DifferentialDriveOdometry m_odometry;
    private DifferentialDriveWheelSpeeds m_wheelSpeeds;
    private Pose2d m_pose;
    private AHRS m_ahrs;
    private SlewRateLimiter rateLimit;
    private double throttle;

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
        m_differentialDrive.setDeadband(0.08);

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

        enableRateLimit();
    }

    public void drive(double speed, double rotation) {
        if (speed < 0.1) {
            m_differentialDrive.curvatureDrive(speed, rotation, true);
        }
        else {
            m_differentialDrive.curvatureDrive(speed, rotation, false);
        }
    }

    public void setThrottle(double throttle) {
        m_differentialDrive.setMaxOutput(throttle);
        this.throttle = throttle;
    }

    public double getThrottle() {
        return throttle;
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

    public class PIDConstants{
        //PID coefficients
        double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel = 0, maxAcc, allowedErr;
        //Some example coefficients obtained from REV; update ASAP with tuned values
        public PIDConstants(){
            //Some example coefficients obtained from REV; update ASAP with tuned values
            kP = 5e-5;       //Proportional coefficient
            kI = 1e-6;       //Integral coefficient
            kD = 0;          //Derivative coefficient
            kIz = 0.5;       //I-zone range of the PIDF controller; the 0.5 is not from REV
            kFF = 0.000156;  //Feed forward constant
            kMaxOutput = 1;  //Maximum PIDF output
            kMinOutput = -1; //Minimum PIDF output

            maxVel = 2000;   //Maximum velocity allowed
            maxAcc = 1500;   //Maximum acceleration allowed
        }
    }

    /*
    public static void setPidConstants(CANPIDController controller, PIDConstants constants) {
        controller.setP(constants.kP);
        controller.setI(constants.kI);
        controller.setD(constants.kD);
        controller.setIZone(constants.kIz);
        controller.setFF(constants.kFF);
        controller.setOutputRange(constants.kMinOutput, constants.kMaxOutput);
     */

        /**
         * Smart Motion coefficients are set on a SparkMaxPIDController object
         *
         * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
         * the pid controller in Smart Motion mode
         * - setSmartMotionMinOutputVelocity() will put a lower bound in
         * RPM of the pid controller in Smart Motion mode
         * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
         * of the pid controller in Smart Motion mode
         * - setSmartMotionAllowedClosedLoopError() will set the max allowed
         * error for the pid controller in Smart Motion mode
         */

    /*
        int smartMotionSlot = 0;
        controller.setSmartMotionMaxVelocity(constants.maxVel, smartMotionSlot); //rpm
        controller.setSmartMotionMinOutputVelocity(constants.minVel, smartMotionSlot); //rpm
        controller.setSmartMotionMaxAccel(constants.maxAcc, smartMotionSlot); //rpm*s^-1
        controller.setSmartMotionAllowedClosedLoopError(constants.allowedErr, smartMotionSlot);
    }
    */

    public void updateSmartDashboardPid(){ //Show PID(F) values for motor controllers:
        SmartDashboard.putNumber("Left P:", m_rightPidController.getP());
        SmartDashboard.putNumber("Left I:", m_rightPidController.getI());
        SmartDashboard.putNumber("Left D:", m_rightPidController.getD());
        SmartDashboard.putNumber("Left Accumulated Error:", m_rightPidController.getIAccum());
        SmartDashboard.putNumber("Right P:", m_rightPidController.getP());
        SmartDashboard.putNumber("Right I:", m_rightPidController.getI());
        SmartDashboard.putNumber("Right D:", m_rightPidController.getD());
        SmartDashboard.putNumber("Right Accumulated Error:", m_rightPidController.getIAccum());
    }

    /*
    public PIDConstants getSmartDashboardPid(){ //does not accept any arguments; the left and right PIDs shouldn't be different
        return PIDConstants()
    }
     */

    @Override
    public void periodic() {
        // Distance
        double leftMeters = m_leftEncoder.getPosition();
        double rightMeters = -m_rightEncoder.getPosition();

        m_pose = m_odometry.update(
                Rotation2d.fromDegrees(getAngle()),
                leftMeters,
                rightMeters);

        m_wheelSpeeds = new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
        SmartDashboard.putNumber("Velocity Left", getLeftVelocity());
        SmartDashboard.putNumber("Velocity Right", getRightVelocity());
        SmartDashboard.putNumber("Throttle", throttle);

        //updateSmartDashboardPid();
    }

    public double getLeftVelocity() {
        return m_leftEncoder.getVelocity();
    }

    public double getRightVelocity() {
        return -m_rightEncoder.getVelocity();
    }

    public PIDController getLeftController() { //for Ramsete controller
        return new PIDController(kP, kI, kD);
    }

    public PIDController getRightController() { //for Ramsete controller
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

    public SlewRateLimiter getRateLimit() {
        return rateLimit;
    }

    public void enableRateLimit() {
        rateLimit = new SlewRateLimiter(2);
    }

    public void disableRateLimit() {
        rateLimit = new SlewRateLimiter(100000000);
    }

    public void setRateLimit(SlewRateLimiter rateLimit) {
        this.rateLimit = rateLimit;
    }

}