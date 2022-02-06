package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.Shooter.*;

public class EnhancedShooter extends CommandBase {
    private Shooter mShooter;
    private Intake mIntake;
    private DriveTrain mDriveTrain;
    private final Limelight mLimelight = Limelight.getInstance();
    private double throttle;
    private double test;

    public EnhancedShooter(Shooter shooter, Intake intake, DriveTrain driveTrain) {
        mShooter = shooter;
        mIntake = intake;
        mDriveTrain = driveTrain;
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(mShooter, mIntake, mLimelight, mDriveTrain);
//        SmartDashboard.putNumber("PID FORWARD LIMELIGHT", 0.3);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Shooter On", true);
        throttle = mDriveTrain.getThrottle();
        mDriveTrain.setThrottle(0.5);
        mShooter.driveMain(kMainSpeed);
        mLimelight.enable();
        mLimelight.setPipeline(0);
    }

    @Override
    public void execute() {
//        test = SmartDashboard.getNumber("PID FORWARD LIMELIGHT", 0.3);
        if (mLimelight.hasTarget() && !mLimelight.isAligned()) {
            mDriveTrain.arcadeDrive(mLimelight.getVertical() * -0.2, mLimelight.getHorizontal() * 0.1);
            return;
        }
        if (!mShooter.isUpToSpeed()) {
            mShooter.driveFeeder(0);
            mShooter.driveAgitator(0);
            mDriveTrain.driveVolts(0,0);
            return;
        }
//        not sure if this drive volts is needed
        mDriveTrain.driveVolts(0,0);
        mShooter.driveFeeder(kFeederSpeed);
        mShooter.driveAgitator(kAgitatorSpeed);
//        intake is broken
//        mIntake.driveMotors(kInnerSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Shooter On", false);
        mLimelight.disable();
        mShooter.driveMain(0);
        mShooter.driveAgitator(0);
        mShooter.driveFeeder(0);
        mIntake.driveMotors(0);
        mDriveTrain.driveVolts(0,0);
        mDriveTrain.setThrottle(throttle);
    }
}