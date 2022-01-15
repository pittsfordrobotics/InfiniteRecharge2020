package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.Intake.kInnerSpeed;
import static frc.robot.Constants.Shooter.*;

public class EnhancedShooter extends CommandBase {
    private final Shooter mShooter;
    private final Intake mIntake;

    public EnhancedShooter(Shooter shooter, Intake intake) {
        mShooter = shooter;
        mIntake = intake;
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(mShooter, mIntake);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Shooter On", true);
    }

    @Override
    public void execute() {
        if (!mShooter.isUpToSpeed()) {
            mShooter.driveMain(kMainSpeed);
            return;
        }
        mShooter.driveFeeder(kFeederSpeed);
        mShooter.driveAgitator(kAgitatorSpeed);
        mIntake.driveMotors(kInnerSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Shooter On", false);
        mShooter.driveMain(0);
        mShooter.driveAgitator(0);
        mIntake.driveMotors(0);
    }
}
