// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final SparkMax m_ShooterMotor = new SparkMax(Constants.ShooterConstants.kShooterMotorPort,
      MotorType.kBrushless);

  public Shooter() {

    SparkMaxConfig ShooterMotorConfig = new SparkMaxConfig();

    ShooterMotorConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake);

    ShooterMotorConfig.closedLoop.pidf(Constants.ShooterConstants.kShooterKp,
        Constants.ShooterConstants.kShooterKi, Constants.ShooterConstants.kShooterKd,
        1.0 / Constants.ShooterConstants.kShooterKv, ClosedLoopSlot.kSlot0);


  }

    // runs motor
    public void runmotor() {
      m_ShooterMotor.set(-.3);
    }

    // stops motor
  public void stopmotor() {
    m_ShooterMotor.set(0.0);
  }

  public Command runShooterMotor() {
    return new StartEndCommand(this::runmotor, this::stopmotor, this);
  }

  public Command runShooterMotor2() {
    return Commands.runOnce(this::runmotor, this);
  }
  public Command stopshooterMoter() {
    return Commands.runOnce(this::stopmotor, this);
  }
  
  public void periodic() {
   }
}
