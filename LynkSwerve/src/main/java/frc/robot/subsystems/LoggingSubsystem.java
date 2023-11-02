// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.Swerve;

public class LoggingSubsystem extends SubsystemBase {
  /** Creates a new LoggingSubsystem. */
  private Swerve s_Swerve;

  public LoggingSubsystem(Swerve s_Swerve) {
    
    this.s_Swerve = s_Swerve;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("SwerveModuleStates", s_Swerve.getModuleStates());
    Logger.recordOutput("Robot Pose", s_Swerve.getPose());
  }
}
