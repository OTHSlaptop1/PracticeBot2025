// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Degrees;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final SparkMax m_ShooterMotor = new SparkMax(Constants.ShooterConstants.kShooterMotorPort,
      MotorType.kBrushless);
  private ShooterSimulation m_OutputSim;
  private SwerveDrive m_Drive;

  private DCMotor m_ShooterGearbox = DCMotor.getNEO(3);

  public Shooter() {

    SparkMaxConfig ShooterMotorConfig = new SparkMaxConfig();

    ShooterMotorConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake);

    ShooterMotorConfig.closedLoop.pidf(Constants.ShooterConstants.kShooterKp,
        Constants.ShooterConstants.kShooterKi, Constants.ShooterConstants.kShooterKd,
        1.0 / Constants.ShooterConstants.kShooterKv, ClosedLoopSlot.kSlot0);


  }

    // runs motor
    public void runmotor() {
      m_ShooterMotor.set(0.4);
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
  public Command stopShooterMotor() {
    return Commands.runOnce(this::stopmotor, this);
  }
  
 public class ShooterSimulation {
  private boolean havePiece = false;

    public ShooterSimulation(DCMotor ShooterGearbox) {
      m_ShooterGearbox = ShooterGearbox;
    }
       // function to addprojectile, called by ejectCoralCommand in sim mode
    public void ejectCoralSim() {
      m_OutputSim.addGamePieceProjectile(m_Drive.getMapleSimDrive().get(), 37.5);
    }

    public void addGamePieceProjectile(SwerveDriveSimulation driveSimulation, double height) {
      SimulatedArena.getInstance().addGamePieceProjectile(new ReefscapeCoralOnFly(
          // Obtain robot position from drive simulation
          driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
          // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
          new Translation2d(0.35, 0),
          // Obtain robot speed from drive simulation
          driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
          // Obtain robot facing from drive simulation
          driveSimulation.getSimulatedDriveTrainPose().getRotation(),
          // The height at which the coral is ejected
          Inches.of(height),
          // The initial speed of the coral
          InchesPerSecond.of(10),
          // The coral is ejected at a 35-degree slope
          Degrees.of(-35)));
    }

  }
}
 


