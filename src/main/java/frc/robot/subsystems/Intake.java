// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Intake extends SubsystemBase {
  private final TalonFX m_IntakeMotor = new TalonFX(Constants.IntakeConstants.kIntakeMotorPort);

  /** Creates a new Intake. */
  public Intake() {

    // Configure the intake motor

    m_IntakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  // Runs motor
  public void runmotor() {
    m_IntakeMotor.set(Constants.IntakeConstants.kIntakeMotorSpeed);
  }
  public Command runIntakeMotor() {
    return new StartEndCommand(this::runmotor, this::stopmotor, this);
  }
  // stops motor
  public void stopmotor() {
    m_IntakeMotor.set(0);
  }
  public Command stopIntakeMotor() {
    return Commands.runOnce(this::stopmotor, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
  