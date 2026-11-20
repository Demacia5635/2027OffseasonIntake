// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

  private final IntakeSubsystem intake;

  public IntakeCommand(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    switch (intake.getState()) {
      case INTAKING, EJECTING, DEPLOYED_IDLE, CLOSED:
        if (intake.getIsCalibrated()) { // only set the deploy angle if it's calibrated
          intake.setDeployAngle(intake.getState().deployAngle);
          if (intake.isAtAngle(intake.getState().deployAngle)) { // if it's at the right angle, start the other motors at the correct duties
            intake.setRollersDuty(intake.getState().rollersDuty);
            intake.setAlignerDuty(intake.getState().alignerDuty);
          } else { 
            intake.setRollersDuty(0); // if the deploy isn't at the right angle, set duty for rollers + aligner to 0
            intake.setAlignerDuty(0);
          }
        } else {
          intake.stopAll(); // if the deploy isn't calibrated, stop all
        }

        break;

      case TESTING:
        intake.setDeployAngle(intake.getTestDeployAngle());
        intake.setRollersDuty(intake.getTestRollersDuty());
        intake.setAlignerDuty(intake.getTestAlignerDuty());
        break;

      case IDLE:
        intake.stopAll();
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopAll();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
