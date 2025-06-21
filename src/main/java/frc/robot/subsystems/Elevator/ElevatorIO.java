// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean leftConnected = false;
    public double leftPositionRad = 0.0;
    public double leftVelocityRadPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;

    public boolean rightConnected = false;
    public double rightPositionRad = 0.0;
    public double rightVelocityRadPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void runVolts(double volts) {}
}
