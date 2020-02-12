/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports.CAN;

public class Spinner extends SubsystemBase {
  private CANSparkMax m_spinnerMotor = new CANSparkMax(CAN.kSpinnerMotor, MotorType.kBrushless);
  /**
   * Creates a new Spinner.
   */
  public Spinner() {
    m_spinnerMotor.restoreFactoryDefaults();
    m_spinnerMotor.getEncoder().setPosition(0);
    m_spinnerMotor.setIdleMode(IdleMode.kBrake);
  }

  public void spin(double spinSpeed) {
    m_spinnerMotor.set(spinSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
