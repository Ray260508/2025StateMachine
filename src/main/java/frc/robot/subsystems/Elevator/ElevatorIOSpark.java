package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkMax leftMotor = new SparkMax(leftMotorId, MotorType.kBrushless);
  private final SparkMax rightMotor = new SparkMax(rightMotorId, MotorType.kBrushless);
  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

  private final SparkClosedLoopController leftController = leftMotor.getClosedLoopController();
  private final SparkClosedLoopController rightController = rightMotor.getClosedLoopController();

  public ElevatorIOSpark() {
    var config = new SparkMaxConfig();
    config
        .inverted(false) // no inverted to both motors so they can use same config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currenLimit)
        .voltageCompensation(12.0);
    config
        .encoder
        .positionConversionFactor(2.0 * Math.PI / motorReduction)
        .velocityConversionFactor((2.0 * Math.PI) / 60.0 / motorReduction)
        .uvwAverageDepth(8)
        .uvwMeasurementPeriod(32);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(kP, kI, kD, kF)
        .outputRange(-1.0, 1.0);

    tryUntilOk(
        leftMotor,
        5,
        () ->
            leftMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        rightMotor,
        5,
        () ->
            rightMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    ifOk(leftMotor, leftEncoder::getPosition, (value) -> inputs.leftPositionRad = value);
    ifOk(leftMotor, leftEncoder::getVelocity, (value) -> inputs.leftVelocityRadPerSec = value);
    ifOk(leftMotor, leftMotor::getOutputCurrent, (value) -> inputs.leftCurrentAmps = value);
    ifOk(
        leftMotor,
        new DoubleSupplier[] {leftMotor::getAppliedOutput, leftMotor::getBusVoltage},
        (values) -> inputs.leftAppliedVolts = values[0] * values[1]); // -1.0~1.0 * 12V

    ifOk(rightMotor, rightEncoder::getPosition, (value) -> inputs.rightPositionRad = value);
    ifOk(rightMotor, rightEncoder::getVelocity, (value) -> inputs.rightVelocityRadPerSec = value);
    ifOk(rightMotor, rightMotor::getOutputCurrent, (value) -> inputs.rightCurrentAmps = value);
    ifOk(
        rightMotor,
        new DoubleSupplier[] {rightMotor::getAppliedOutput, rightMotor::getBusVoltage},
        (values) -> inputs.rightAppliedVolts = values[0] * values[1]); // -1.0~1.0 * 12V
  }

  @Override
  public void runVolts(double volts) {
    leftMotor.setVoltage(Math.max(-12.0, Math.min(12.0, volts)));
    rightMotor.setVoltage(Math.max(-12.0, Math.min(12.0, volts)));
  }

  //   @Override
  //   public void runPosition(double positionRotation) {
  //     leftController.setReference(
  //         MathUtil.inputModulus(positionRotation, minPositionRotation, maxPositionRotation),
  //         ControlType.kPosition);
  //     rightController.setReference(
  //         MathUtil.inputModulus(positionRotation, minPositionRotation, maxPositionRotation),
  //         ControlType.kPosition);
  //   }
}
