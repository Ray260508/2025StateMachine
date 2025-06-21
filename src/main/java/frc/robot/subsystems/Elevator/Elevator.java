package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final ProfiledPIDController pidController = new ProfiledPIDController(
      simkP,
      simkI,
      simkD,
      new TrapezoidProfile.Constraints(simMaxVelocity, simMaxAcceleration));
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(simkS, simkG, simkV, simkA);

  public Elevator(ElevatorIO io) {
    this.io = io;
    pidController.reset(0.5 / simDrumRadiusMeters);
    pidController.setGoal(0.5 / simDrumRadiusMeters);
    pidController.setTolerance(0.05);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    double measuredPosition = inputs.leftPositionRad;
    double pid = pidController.calculate(measuredPosition);
    double ff = feedforward.calculate(pidController.getSetpoint().velocity);

    io.runVolts(pid + ff);
  }

  @AutoLogOutput(key = "Elevator/Position")
  public double getPositionMeters() {
    return inputs.leftPositionRad * simDrumRadiusMeters;
  }

  @AutoLogOutput(key = "Elevator/AppliedVolts")
  public double getAppliedVolts() {
    return inputs.leftAppliedVolts;
  }

  public void setPosition(double positionMeters) {
    pidController.setGoal(positionMeters / simDrumRadiusMeters);
  }
}
