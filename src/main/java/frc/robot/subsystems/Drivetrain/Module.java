// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.DrivetrainConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.ModuleConstants;

import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    this.io.resetEncoders();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    io.setState(state);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setState(new SwerveModuleState(0, getAngle()));
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return getPosition().angle;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return inputs.position;
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return inputs.state;
  }

  /** Returns the desired module state (turn angle and drive velocity). */
  public SwerveModuleState getDesiredState() {
    return inputs.desiredState;
  }

  public void resetEncoders() {
    io.resetEncoders();
  }
}
