/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Ports {
        public static final class CAN {
<<<<<<< HEAD
            public static final int kDriveLeftPrimary = 11;
            public static final int kDriveLeftFollower = 12;
            public static final int kDriveRightPrimary = 13;
            public static final int kDriveRightFollower = 14;
=======
            public static final int kTelescopingArm = 6;
            public static final int kWinchLeft = 7;
            public static final int kWinchRight = 8;            


            public static final int kLeftPrimary = 1;
            public static final int kLeftFollower = 4;

            public static final int kRightPrimary = 2;
            public static final int kRightFollower = 3;
>>>>>>> made the basic climber code.
        }
    }

    public static final class PWM {

    }

    public static final class DIO {

    }

    public static final class AIO {

    }

    public static final class Drive {
        public static final double kP = 3;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0.174;
        public static final double kV = 2.76;
        public static final double kA = 0.326;

        public static final double kMaxVelocityMetersPerSecond = 1.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;

        public static final double kGearRatio = 10.71;
        public static final double kWheelDiameterMeters = 0.1524;
        public static final DifferentialDriveKinematics kKinematics = new DifferentialDriveKinematics(0.563);
    }

    public static final class Intake {

    }

    public static final class Shooter {

    }

    public static final class Climber {

    }

    public static final class Spinner {

    }
}
