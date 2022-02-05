package frc.robot.commands.limelight;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class LimelightDrive extends CommandBase {
    private DriveTrain driveTrain;
    private final Limelight limelight = Limelight.getInstance();

    public LimelightDrive(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain, this.limelight);
    }

    private class LimelightPID {
        public class ShuffleboardManager{
            ShuffleboardTab tab = Shuffleboard.getTab("Limelight");
            NetworkTableEntry kTEntry = tab.addPersistent("kT", limelightPID.constants.kT).getEntry();
            NetworkTableEntry setpointEntry = tab.addPersistent("Setpoint", limelightPID.constants.setpoint).getEntry();
            NetworkTableEntry kPEntry = tab.addPersistent("kP", limelightPID.constants.kP).getEntry();
            NetworkTableEntry kIEntry = tab.addPersistent("kI", limelightPID.constants.kI).getEntry();
            NetworkTableEntry kDEntry = tab.addPersistent("kD", limelightPID.constants.kD).getEntry();
            NetworkTableEntry kIzEntry = tab.addPersistent("kIz", limelightPID.constants.kIz).getEntry();
            NetworkTableEntry kFFEntry = tab.addPersistent("kFF", limelightPID.constants.kFF).getEntry();
            NetworkTableEntry kMaxVelEntry = tab.addPersistent("Maximum Acceleration", limelightPID.constants.kMaxVel).getEntry();
            NetworkTableEntry kAllowedErrEntry = tab.addPersistent("Allowed Error", limelightPID.constants.kAllowedErr).getEntry();

            NetworkTableEntry pEntry = tab.add("Error (P)", limelightPID.P).getEntry();
            NetworkTableEntry iEntry = tab.add("Accumulated Error (I)", limelightPID.I).getEntry();
            NetworkTableEntry dEntry = tab.add("Rate (D)", limelightPID.D).getEntry();
            NetworkTableEntry outputEntry = tab.add("Output", limelightPID.output).getEntry();

            public void update() {
                limelightPID.constants.kT = kTEntry.getDouble(0);
                limelightPID.constants.setpoint = setpointEntry.getDouble(0);
                limelightPID.constants.kP = kPEntry.getDouble(0);
                limelightPID.constants.kI = kIEntry.getDouble(0);
                limelightPID.constants.kD = kDEntry.getDouble(0);
                limelightPID.constants.kIz = kIzEntry.getDouble(0);
                limelightPID.constants.kFF = kFFEntry.getDouble(0);
                limelightPID.constants.kMaxVel = kMaxVelEntry.getDouble(0);
                limelightPID.constants.kAllowedErr = kAllowedErrEntry.getDouble(0);

                pEntry.setDouble(P);
                iEntry.setDouble(I);
                dEntry.setDouble(D);
                outputEntry.setDouble(output);
            }
        }

        ShuffleboardManager limeBoard = new ShuffleboardManager();
        double P = 0;
        double I = 0;
        double D = 0;
        double prevOutput = 0;
        double output = 0;

        private class PIDConstants {
            //placeholder values
            double kT = 0.05;      //period between each output
            double setpoint = 0.1;   //desired position
            double kP = 0.5;       //position weight
            double kI = 0.1;       //integral weight
            double kD = 0.1;       //derivative weight
            double kIz = 0.5;      //maximum accumulated error = 0 +-kIz
            double kFF = 0;        //feedforward "guesstimate"

            double kMaxOutput = 1;
            double kMinOutput = -1;
            double kMaxVel = 0.5;
            double kAllowedErr = 0.1;
        }

        PIDConstants constants = new PIDConstants();
        PIDController controller = new PIDController(constants.kP, constants.kI, constants.kD, constants.kT);

        public LimelightPID() {
            controller.setPID(constants.kP, constants.kI, constants.kD);
            controller.setSetpoint(constants.setpoint);
            controller.disableContinuousInput();
        }

        private void updatePID() {
            limeBoard.update();
            controller.setP(constants.kP);
            controller.setI(constants.kI);
            controller.setD(constants.kD);
            controller.setIntegratorRange(-constants.kIz, constants.kIz);
            controller.setTolerance(constants.kAllowedErr, constants.kMaxVel/5);
        }

        private double getOutput() {
            prevOutput = output;
            output = controller.calculate(limelight.getVertical());
            return MathUtil.clamp(Math.abs(output - prevOutput) > constants.kMaxVel ? output : prevOutput + constants.kMaxVel*Math.signum(output - prevOutput), constants.kMinOutput, constants.kMaxOutput);
        }

        private void reset(){
            limelightPID.controller.reset();
        }
    }

    private LimelightPID limelightPID = new LimelightPID();

    @Override
    public void initialize() {
        limelight.enable();
    }

    @Override
    public void execute() {
        limelightPID.updatePID();
        driveTrain.setThrottle(0.2);
        driveTrain.drive(limelightPID.getOutput(), limelight.getHorizontal() * 0.5);
    }

    @Override
    public boolean isFinished() {
        return limelightPID.controller.atSetpoint() && limelight.getHorizontal() < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        limelight.disable();
        driveTrain.driveVolts(0, 0);
        limelightPID.reset();
    }
}