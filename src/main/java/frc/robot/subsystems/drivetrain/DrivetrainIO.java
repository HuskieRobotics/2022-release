// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Intake subsystem hardware interface. */
public interface DrivetrainIO {
  /**
   * Contains all of the input data received from hardware. More can be logged when we have our own
   * swerve library code with access to the motors and encoders
   */
  public static class DrivetrainIOInputs implements LoggableInputs {
    double frontLeftDriveVelocity = 0.0;
    double frontLeftSteerAngle = 0.0;

    double frontRightDriveVelocity = 0.0;
    double frontRightSteerAngle = 0.0;

    double backLeftDriveVelocity = 0.0;
    double backLeftSteerAngle = 0.0;

    double backRightDriveVelocity = 0.0;
    double backRightSteerAngle = 0.0;

    double yaw = 0.0;

    double limelightTX = 0.0;
    double limelightTY = 0.0;
    double limelightTV = 0.0;

    public void toLog(LogTable table) {
      table.put("FrontLeftDriveVelocity", frontLeftDriveVelocity);
      table.put("FrontLeftSteerAngle", frontLeftSteerAngle);
      table.put("FrontRightDriveVelocity", frontRightDriveVelocity);
      table.put("FrontRightSteerAngle", frontRightSteerAngle);
      table.put("BackLeftDriveVelocity", backLeftDriveVelocity);
      table.put("BackLeftSteerAngle", backLeftSteerAngle);
      table.put("BackRightDriveVelocity", backRightDriveVelocity);
      table.put("BackRightSteerAngle", backRightSteerAngle);
      table.put("Yaw", yaw);
      table.put("LimelightTX", limelightTX);
      table.put("LimelightTY", limelightTY);
      table.put("LimelightTV", limelightTV);
    }

    public void fromLog(LogTable table) {
      frontLeftDriveVelocity = table.getDouble("FrontLeftDriveVelocity", frontLeftDriveVelocity);
      frontLeftSteerAngle = table.getDouble("FrontLeftSteerAngle", frontLeftSteerAngle);
      frontRightDriveVelocity = table.getDouble("FrontRightDriveVelocity", frontRightDriveVelocity);
      frontRightSteerAngle = table.getDouble("FrontRightSteerAngle", frontRightSteerAngle);
      backLeftDriveVelocity = table.getDouble("BackLeftDriveVelocity", backLeftDriveVelocity);
      backLeftSteerAngle = table.getDouble("BackLeftSteerAngle", backLeftSteerAngle);
      backRightDriveVelocity = table.getDouble("BackRightDriveVelocity", backRightDriveVelocity);
      backRightSteerAngle = table.getDouble("BackRightSteerAngle", backRightSteerAngle);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DrivetrainIOInputs inputs) {}

  /** Set the swerve modules to the specified states. */
  public default void setSwerveModules(SwerveModuleState[] states) {}

  /** Set the swerve modules to the specified states with feed forward. */
  public default void setSwerveModulesWithFeedForward(SwerveModuleState[] states) {}

  /** Run the flywheel closed loop at the specified velocity. */
  public default void setVelocity(double velocity) {}
}
