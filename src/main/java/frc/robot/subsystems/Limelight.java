package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    //NetworkTable key names can be found at https://docs.limelightvision.io/en/latest/networktables_api.html
    //Outputs from Limelight:
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry ts = table.getEntry("ts");
    NetworkTableEntry tl = table.getEntry("tl");
    NetworkTableEntry tshort = table.getEntry("tshort");
    NetworkTableEntry tlong = table.getEntry("tlong");
    NetworkTableEntry thor = table.getEntry("thor");
    NetworkTableEntry tvert = table.getEntry("tvert");
    NetworkTableEntry getpipe = table.getEntry("getpipe");
    //NetworkTableEntry camtran = table.getEntry("camtran"); //not used, 6DOF 3D position solution (translation + YPR)

    //Inputs to Limelight:
    NetworkTableEntry ledMode = table.getEntry("ledMode");
    NetworkTableEntry camMode = table.getEntry("camMode");
    NetworkTableEntry pipeline = table.getEntry("pipeline");
    NetworkTableEntry stream = table.getEntry("stream");
    NetworkTableEntry snapshot = table.getEntry("snapshot");


    public static class PeriodicIO { //cursed Java struct equivalent w/ an excessive amount of boilerplate code
        //outputs from limelight
        public int ledMode = 0; //0: pipeline default, 1: force off, 2: force blink, 3: force on
        public int camMode = 0; //0: vison processing, 1: driver camera (increases exposure, disables vision processing)
        public int pipeline = 0; //0-9; sets limelight pipeline
        public int stream = 1; //0: side-by-side streams for webcam attached to limelight, 1: secondary camera in lower right of primary camera stream; 2: reverse of 1
        public int snapshot = 0; //0: no snapshots; 1: 2 snapshots per second

        //inputs to limelight
        public boolean hasTarget;
        public double x, y, area, skew, latency;
        public int width, length, horizontal, vertical, setLedMode, setPipeline;
    }

    private void updateConstants(){
        if(mPeriodicIO.setLedMode != mPeriodicIO.ledMode || mPeriodicIO.setPipeline != mPeriodicIO.pipeline){
            ledMode.setNumber(mPeriodicIO.ledMode);
            camMode.setNumber(mPeriodicIO.camMode);
            pipeline.setNumber(mPeriodicIO.pipeline);
            stream.setNumber(mPeriodicIO.stream);
            snapshot.setNumber(mPeriodicIO.snapshot);
        }
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    public double getLatency() {
        return mPeriodicIO.latency;
    }

    @Override
    public void periodic() {
        updateConstants();

        mPeriodicIO.hasTarget = tv.getDouble(0) == 1.0; //Number of valid targets; 0 or 1 converted to false or true
        mPeriodicIO.x = tx.getDouble(0.0); //Horizontal offset from crosshair to target; ±27° in LL1, ±29.8° in LL2 
        mPeriodicIO.y = ty.getDouble(0.0); //Vertical offset from crosshair to target; ±20.5° in LL1, ±24.85° in LL2
        mPeriodicIO.area = ta.getDouble(0.0); //Target area; 0% to 100%
        mPeriodicIO.skew = ts.getDouble(0.0); //Skew or rotation of the target, from -90 degrees to 0 degrees
        mPeriodicIO.latency = tl.getDouble(0.0) + 11; //In ms, add >= 11 ms for image capture latency
        mPeriodicIO.width = (int) tshort.getDouble(0); //Length of shortest side of bounding box, in pixels
        mPeriodicIO.length = (int) tlong.getDouble(0); //Length of longest side of bounding box, in pixels
        mPeriodicIO.horizontal = (int) thor.getDouble(0); //Length of horizontal side of bounding box, 0-320 pixels
        mPeriodicIO.vertical = (int) tvert.getDouble(0); //Length of vertical side of bounding box, 0-320 pixels
        mPeriodicIO.pipeline = (int) getpipe.getDouble(0); //Pipeline used, between 0 and 9

        SmartDashboard.putBoolean("LimelightHasTarget", mPeriodicIO.hasTarget);
        SmartDashboard.putNumber("LimelightX", mPeriodicIO.x);
        SmartDashboard.putNumber("LimelightY", mPeriodicIO.y);
        SmartDashboard.putNumber("LimelightArea", mPeriodicIO.area);
        SmartDashboard.putNumber("LimelightSkew", mPeriodicIO.skew);
        SmartDashboard.putNumber("LimelightLatency", mPeriodicIO.latency);
        SmartDashboard.putNumber("LimelightPipeline", mPeriodicIO.setPipeline);
    }
}
