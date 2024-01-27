// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoSubsystem extends SubsystemBase {
  /** Creates a new AutoSubsystem. */
  public boolean in = false;
  public boolean out = false;

  public AutoSubsystem() {}

  public void inTrue(){
    in = false;
  }

  public void shootTrue(){
    out = true;
  }

  public void inFalse(){
    in = false;
  }

  public void shootFalse(){
    out = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
