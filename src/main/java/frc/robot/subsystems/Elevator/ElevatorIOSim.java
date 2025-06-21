package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ElevatorIOSim implements ElevatorIO {

  private final DCMotor motor = DCMotor.getKrakenX60(2);
  private final ElevatorSim sim =
      new ElevatorSim(
          motor,
          simMotorReduction,
          simCarriageMassKg,
          simDrumRadiusMeters,
          simMinHeightMeters,
          simMaxHeightMeters,
          true,
          0.5);

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.update(0.02); // update simulation at 20ms timestep

    inputs.leftConnected = true;
    inputs.leftPositionRad = sim.getPositionMeters() / simDrumRadiusMeters;
    inputs.leftVelocityRadPerSec = sim.getVelocityMetersPerSecond() / simDrumRadiusMeters;
    inputs.leftAppliedVolts = appliedVolts;
    inputs.leftCurrentAmps = sim.getCurrentDrawAmps();

    // If simulating a second motor, just mirror the values here
    inputs.rightConnected = true;
    inputs.rightPositionRad = inputs.leftPositionRad;
    inputs.rightVelocityRadPerSec = inputs.leftVelocityRadPerSec;
    inputs.rightAppliedVolts = inputs.leftAppliedVolts;
    inputs.rightCurrentAmps = inputs.leftCurrentAmps;

    // Update battery voltage simulation
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));
  }

  @Override
  public void runVolts(double volts) {
    appliedVolts = Math.max(-12.0, Math.min(12.0, volts));
    sim.setInput(appliedVolts);
  }
}
