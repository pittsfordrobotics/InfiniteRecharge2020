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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TargetColors;
import frc.robot.Constants.Ports.CAN;

public class Spinner extends SubsystemBase {
  private CANSparkMax m_spinnerMotor = new CANSparkMax(CAN.kSpinnerMotor, MotorType.kBrushless);
  private ColorSensorV3 m_colorSensor = new ColorSensorV3(Port.kOnboard);
  private ColorMatch m_colorMatcher = new ColorMatch();
  /**
   * Creates a new Spinner.
   */
  public Spinner() {
    m_spinnerMotor.restoreFactoryDefaults();
    m_spinnerMotor.getEncoder().setPosition(0);
    m_spinnerMotor.setIdleMode(IdleMode.kBrake);
    
    m_colorMatcher.addColorMatch(TargetColors.kBlueTarget);
    m_colorMatcher.addColorMatch(TargetColors.kGreenTarget);
    m_colorMatcher.addColorMatch(TargetColors.kRedTarget);
    m_colorMatcher.addColorMatch(TargetColors.kYellowTarget);
  }
  public Color readColor() {
    Color color = m_colorSensor.getColor();
    ColorMatchResult result = m_colorMatcher.matchClosestColor(color);
    return result.color;
  }

  public void spin(double spinSpeed) {
    m_spinnerMotor.set(spinSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
