// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.robot.commands.CalibrationCommand;
import frc.robot.constants.IntakeConstants.IntakeState;

import static frc.robot.constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFXMotor rollersMotor;
  private final TalonFXMotor alignerMotor;

  private final TalonFXMotor deployMotor;
  private final LimitSwitch deployLimitSwitch;

  private IntakeState state;
  private boolean isCalibrated = RobotBase.isSimulation(); // if its a simulation, set is calibrated to true

  private double testDeployAngle = 0;
  private double testRollersDuty = 0;
  private double testAlignerDuty = 0;

  public IntakeSubsystem() {
    rollersMotor = new TalonFXMotor(ROLLERS_MOTOR_CONFIG);
    alignerMotor = new TalonFXMotor(ALIGNER_MOTOR_CONFIG);
    deployMotor = new TalonFXMotor(DEPLOY_MOTOR_CONFIG);
    deployLimitSwitch = new LimitSwitch(LIMIT_SWITCH_CONFIG);

    SmartDashboard.putData("Intake Subsystem", this);
    SmartDashboard.putData("Calibration Command", new CalibrationCommand(this));
    SmartDashboard.putData("Set brake deploy", new InstantCommand(() -> {setDeployNeutralMode(true);}).ignoringDisable(true));
    SmartDashboard.putData("Set coast deploy", new InstantCommand(() -> {setDeployNeutralMode(false);}).ignoringDisable(true));
    
    addStateChooser();
  }

  public void addStateChooser() {
    SendableChooser<IntakeState> stateChooser = new SendableChooser<>();
    stateChooser.setDefaultOption(IntakeState.IDLE.name(), IntakeState.IDLE); // default
    for (IntakeState state : IntakeState.values()) {
      stateChooser.addOption(state.name(), state);
    }
    stateChooser.onChange(this::setState);
    SmartDashboard.putData("State Chooser", stateChooser);
  }

  // Set power for all motors
  public void setRollersDuty(double duty) {
    rollersMotor.setDuty(duty);
  }

  public void setAlignerDuty(double duty) {
    alignerMotor.setDuty(duty);
  }

  public void setDeployDuty(double duty) {
    deployMotor.setDuty(duty);
  }

  public void setDeployPosition(double position) {
    deployMotor.setMotion(position);
  }

  public void setDeployAngle(double angle) {
    double clampedAngle = MathUtil.clamp(angle, CLOSED_DEPLOY_ANGLE, OPEN_DEPLOY_ANGLE); // limit the angle to only between the closed and open angle
    setDeployPosition(clampedAngle); 
  }

  public void setDeployNeutralMode(boolean isBrake) {
    deployMotor.setNeutralMode(isBrake);
  }

  public void setCalibrated() {
    deployMotor.setEncoderPosition(CLOSED_DEPLOY_ANGLE);
    isCalibrated = true;
    setState(IntakeState.CLOSED);
  }

  public boolean getIsCalibrated() {
    return isCalibrated;
  }

  public boolean isAtAngle(double deployAngle) {
    if (RobotBase.isSimulation()) { // if it's a simulation, just return true
    return true;
    }
    double currentAngle = deployMotor.getCurrentAngle();
    return (Math.abs(currentAngle - deployAngle) < ANGLE_TOLORANCE);
  }

  // Getters

  public double getRollersVelocity() {
    return rollersMotor.getCurrentVelocity();
  }

  public double getIntakeDeployAngle() {
    return Math.toDegrees(deployMotor.getCurrentPosition());
  }

  public boolean isDeployClosed() {
    return !deployLimitSwitch.get(); // if it's not pressed it returns true so just invert it
  }

  // Get test values

  public double getTestDeployAngle() {
    return testDeployAngle;
  }

  public double getTestRollersDuty() {
    return testRollersDuty;
  }

  public double getTestAlignerDuty() {
    return testAlignerDuty;
  }
  

  // Stop
  public void stopRollers() {
    rollersMotor.stop();
  }

  public void stopAligner() {
    alignerMotor.stop();
  }

  public void stopDeploy() {
    deployMotor.stop();
  }

  public void stopAll() {
    stopDeploy();
    stopRollers();
    stopAligner();
  }


  // Set state and reset values for TESTING
  public void resetTestValues() {
    testDeployAngle = 0;
    testRollersDuty = 0;
    testAlignerDuty = 0;
  }
  public void setState(IntakeState newState) {
    if (this.state == IntakeState.TESTING && newState!= IntakeState.TESTING) { // if you change a state after you're in testing, it resets the test values back to 0
      resetTestValues();
    } 
    state = newState;
  }

  public IntakeState getState() {
    return state;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType("IntakeSubsystem");

    builder.addDoubleProperty("Target Deploy Angle (Degrees)",() -> Math.toDegrees(testDeployAngle), (angle) -> testDeployAngle = Math.toRadians(angle));
    builder.addDoubleProperty("Target Rollers Duty",() -> testRollersDuty, (duty) -> testRollersDuty = duty);
    builder.addDoubleProperty("Target Aligner Duty",() -> testAlignerDuty, (duty) -> testAlignerDuty = duty);
    builder.addBooleanProperty("Is Calibratred", () -> getIsCalibrated(), null);
  }
  @Override
  public void periodic() {
  }
}
