/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

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
            public static final int kDriveLeftPrimary = 11;
            public static final int kDriveLeftFollower = 12;
            public static final int kDriveRightPrimary = 13;
            public static final int kDriveRightFollower = 14;

            public static final int kClimberTelescopingArm = 21;
            public static final int kClimberWinch = 22;

            public static final int kShooterMain = 31;
            public static final int kShooterFeeder = 32;
            public static final int kShooterAgitator = 33;
            
            public static final int kSpinnerLeftRight = 41;
            public static final int kSpinnerUpDown = 42;

            public static final int kIntakeInner = 51;
            // removed
            // public static final int kIntakeOuter = 52;
        }
    }

    public static final class PCM {

    }

    public static final class PWM {
        public static final int kIntakeLeft = 0;
        public static final int kIntakeRight = 2;
    }

    public static final class DIO {

    }

    public static final class AIO {

    }

    public static final class Auto {
        public static final TrajectoryConfig kConfig = new TrajectoryConfig(
            Drive.kMaxVelocityMetersPerSecond,
            Drive.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Drive.kKinematics)
            .addConstraint(
                new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA),
                Drive.kKinematics,
                10
            )
        );

        public static final TrajectoryConfig kReversedConfig = new TrajectoryConfig(
            Drive.kMaxVelocityMetersPerSecond,
            Drive.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Drive.kKinematics)
            .setReversed(true)
            .addConstraint(
                new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA),
                Drive.kKinematics,
                10
            )
        );
    }

    public static final class Drive {
        public static final double kP = 3;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0.191;
        public static final double kV = 2.72;
        public static final double kA = 0.492;

        public static final double kMaxVelocityMetersPerSecond = 1.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;

        public static final double kGearRatio = 10.71;
        public static final double kWheelDiameterMeters = 0.1524;
        public static final DifferentialDriveKinematics kKinematics = new DifferentialDriveKinematics(0.644);
    }

    public static final class Intake {
        public static final double kInnerSpeed = 3;
    }

    public static final class Shooter {
        public static final double kP = 0.0003;
        public static final double kF = 0.00018;
        public static final int kMainSpeed = 2955;
        public static final double kAgitatorSpeed = 0.3;
        public static final double kFeederSpeed = 3;
        public static final int kSpeedThreshold = 2800;
    }

    public static final class Climber {
        public static final int kMaxTelescopingArmPosition = 175;
        public static final double kTelescopingArmSpeed = 0.25;
        public static final double kWinchSpeed = 0.4;
    }

    public static final class Spinner {
        // public static final Color kBlueTarget = ColorMatch.makeColor(0.17, 0.43, 0.41);
        // public static final Color kGreenTarget = ColorMatch.makeColor(0.23, 0.50, 0.27);
        // public static final Color kRedTarget = ColorMatch.makeColor(0.35, 0.43, 0.23);
        // public static final Color kYellowTarget = ColorMatch.makeColor(0.30, 0.52, 0.18);

        public static final double kMaxUpDownPosition = -44; // Negative is up
        public static final double kLeftRightSpeed = 0.5;
        public static final double kUpDownResetSpeed = 0.1;
        public static final double kUpDownSpeed = 0.2;
    }

    public static final class Limelight {
//        change these numbers
//        represents the angle between robot and limelight
        public static final double offsetAngle = 0;
//        height to limelight
        public static final double robotHeight = 0;
//        target height
        public static final double targetHeight = 0;

    }
}
