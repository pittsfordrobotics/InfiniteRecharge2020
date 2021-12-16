package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class LimelightDrive extends CommandBase {
    private final DriveTrain driveTrain;
    private final Limelight limelight = Limelight.getInstance();

    public LimelightDrive(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain, this.limelight);
    }

    @Override
    public void initialize() {
        limelight.enable();
    }

    @Override
    public void execute() {
        driveTrain.setThrottle(0.2);
        driveTrain.drive(limelight.getVertical() * -0.1, limelight.getHorizontal() * -0.1);
    }

    @Override
    public boolean isFinished() {
        return limelight.getVertical() < 0.2 && limelight.getHorizontal() < 0.2;
    }

    @Override
    public void end(boolean interrupted) {
        limelight.disable();
        driveTrain.driveVolts(0, 0);
    }
}