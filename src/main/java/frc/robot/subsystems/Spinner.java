/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Spinner.*;
import static frc.robot.Constants.Ports.*;

public class Spinner extends SubsystemBase {
    private CANSparkMax m_spinnerLeftRight = new CANSparkMax(CAN.kSpinnerLeftRight, MotorType.kBrushless);
    private ColorSensorV3 m_colorSensor = new ColorSensorV3(Port.kOnboard);
    private ColorMatch m_colorMatcher = new ColorMatch();

    /**
     * Creates a new Spinner.
     */
    public Spinner() {
        SmartDashboard.putNumber("Spinner Speed", 0.5);

        m_spinnerLeftRight.restoreFactoryDefaults();
        m_spinnerLeftRight.getEncoder().setPosition(0);
        m_spinnerLeftRight.setIdleMode(IdleMode.kBrake);

        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);
    }

    public Color readColor() {
        Color color = m_colorSensor.getColor();
        ColorMatchResult result = m_colorMatcher.matchClosestColor(color);
        return result.color;
    }

    public void spin(double spinSpeed) {
        m_spinnerLeftRight.set(spinSpeed);
    }

    @Override
    public void periodic() {
    }
}
