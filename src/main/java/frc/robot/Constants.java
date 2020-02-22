/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

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

            public static final int kIntakeInner = 51;
            public static final int kIntakeOuter = 52;
        }
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
        public static final double kMaxActuatorPulse = 2;
        public static final double kCenterActuatorPulse = 1.5;
        public static final double kMinActuatorPulse = 1;
        public static final double kMaxActuatorDeadband = 0.05;
        public static final double kMinActuatorDeadband = 0;
    }

    public static final class Shooter {
        public static final double kP = 0.0003;
        public static final double kF = 0.00018;
        public static final double kSpeedDelta = 100;
    }

    public static final class Climber {

    }

    public static final class Spinner {
        public static final Color kBlueTarget = ColorMatch.makeColor(0.17, 0.43, 0.41);
        public static final Color kGreenTarget = ColorMatch.makeColor(0.23, 0.50, 0.27);
        public static final Color kRedTarget = ColorMatch.makeColor(0.35, 0.43, 0.23);
        public static final Color kYellowTarget = ColorMatch.makeColor(0.30, 0.52, 0.18);

    }
}
