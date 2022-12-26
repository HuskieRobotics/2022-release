package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.storage.Storage;

/**
 * This command, when executed, instructs the storage subsystem to run its belt until cargo is
 * detected at the shooter-end of the storage at which point it continues to run the belt for a
 * period of time in the intake direction to ensure the cargo is positioned against the flywheel.
 * After that it runs the belt in the outtake direction in order to move it away from the flywheel
 * such that when the flywheel begins spinning the cargo will not be inadvertantly shot.
 *
 * <p>Requires: the storage subsystem
 *
 * <p>Finished When: the flywheel velocity is at the setpoint
 *
 * <p>At End: leaves the flywheel spinning
 */
public class SortStorageCommand extends CommandBase {
  private Storage storage;
  private int indexingDelayCount;

  /**
   * Constructs a new SortStorageCommand object.
   *
   * @param storage the storage subsystem this command will control
   */
  public SortStorageCommand(Storage storage) {
    this.storage = storage;
    this.addRequirements(this.storage);
  }

  /**
   * This method is invoked once when this command is scheduled. It initializes the indexing delay
   * counter. It is critical that this initialization occurs in this method and not the constructor
   * as this command is constructed once when the RobotContainer is created, but this method is
   * invoked each time this command is scheduled.
   */
  @Override
  public void initialize() {
    indexingDelayCount = 0;
  }

  /**
   * This method will be invoked every iteration of the Command Scheduler.
   *
   * <p>If cargo is detected at the collector end of the storage and not at the shooter end, it runs
   * the storage belt in the intake direction.
   *
   * <p>If cargo is detected at the shooter end of the storage, it continues to run the storage in
   * the intake direction for the desired number of iterations. After that, it reverses the
   * direction of the storage belt for the desired number of iterations. After that, it stops the
   * storage belt.
   */
  @Override
  public void execute() {
    if (!this.storage.isShooterSensorUnblocked()) {
      indexingDelayCount++;
      if (indexingDelayCount == 1) {
        this.storage.enableStorage();
      }
      if (indexingDelayCount == StorageConstants.INDEXING_FORWARD_DELAY) {
        this.storage.setStoragePower(StorageConstants.OUTTAKE_POWER);
      } else if (indexingDelayCount
          > StorageConstants.INDEXING_FORWARD_DELAY + StorageConstants.INDEXING_BACKWARD_DURATION) {
        this.storage.disableStorage();
      }
    } else if (!this.storage.isCollectorSensorUnblocked()
        && this.storage.isShooterSensorUnblocked()) {
      this.storage.enableStorage();
      indexingDelayCount = 0;
    }
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the storage.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    this.storage.disableStorage();
  }

  /**
   * This method is invoked at the end of each Command Scheduler iteration. It returns true when
   * cargo is detected at both the shooter and collector end of the storage and the indexing of the
   * cargo at the shooter end has completed. It is critical that this command doesn't finish until
   * the indexing is compelte or else the cargo may still be touching the flywheel and then
   * inadvertantly shot.
   */
  @Override
  public boolean isFinished() {
    return !this.storage.isCollectorSensorUnblocked()
        && !this.storage.isShooterSensorUnblocked()
        && indexingDelayCount
            > StorageConstants.INDEXING_FORWARD_DELAY + StorageConstants.INDEXING_BACKWARD_DURATION;
  }
}
