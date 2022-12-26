package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.JoystickConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.StorageConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.VisionBox;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.collector.CollectorIO;
import frc.robot.subsystems.collector.CollectorIOTalonSRX;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.drivetrain.DrivetrainIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.robot.subsystems.pneumatics.PneumaticsIO;
import frc.robot.subsystems.pneumatics.PneumaticsIORev;
import frc.robot.subsystems.secondary_arm.SecondaryArm;
import frc.robot.subsystems.secondary_arm.SecondaryArmIO;
import frc.robot.subsystems.secondary_arm.SecondaryArmSolenoid;
import frc.robot.subsystems.storage.Storage;
import frc.robot.subsystems.storage.StorageIO;
import frc.robot.subsystems.storage.StorageIOTalonSRX;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final JoystickButton[] operatorButtons;
  private final JoystickButton[] joystickButtons0;
  private final JoystickButton[] joystickButtons1;
  private final JoystickButton[] xboxButtons;

  private final Joystick joystick0 = new Joystick(0);
  private final Joystick joystick1 = new Joystick(1);
  private final Joystick operatorConsole = new Joystick(2);
  private final XboxController xboxController = new XboxController(3);

  private static RobotContainer robotContainer = new RobotContainer();

  private final Drivetrain drivetrain;
  private final Storage storage;
  private final Collector collector;
  private final Flywheel flywheel;
  private final SecondaryArm secondMechanism;
  private final Elevator elevator;
  private final Pneumatics pneumatics;
  private final VisionBox visionBox;

  // A chooser for autonomous commands
  SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  private RobotContainer() {

    // create real or replay subsystems
    if (Constants.getMode() != Mode.REPLAY) {
      drivetrain = new Drivetrain(new DrivetrainIOTalonFX());
      storage = new Storage(new StorageIOTalonSRX());
      collector = new Collector(new CollectorIOTalonSRX());
      flywheel = new Flywheel(new FlywheelIOTalonFX());
      secondMechanism = new SecondaryArm(new SecondaryArmSolenoid());
      elevator = new Elevator(new ElevatorIOTalonFX());
      pneumatics = new Pneumatics(new PneumaticsIORev());
    } else {
      drivetrain = new Drivetrain(new DrivetrainIO() {});
      storage = new Storage(new StorageIO() {});
      collector = new Collector(new CollectorIO() {});
      flywheel = new Flywheel(new FlywheelIO() {});
      secondMechanism = new SecondaryArm(new SecondaryArmIO() {});
      elevator = new Elevator(new ElevatorIO() {});
      pneumatics = new Pneumatics(new PneumaticsIO() {});
    }

    visionBox = new VisionBox(drivetrain);

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    // buttons use 1-based indexing such that the index matches the button number
    this.joystickButtons0 = new JoystickButton[13];
    this.joystickButtons1 = new JoystickButton[13];
    this.operatorButtons = new JoystickButton[13];
    this.xboxButtons = new JoystickButton[11];

    for (int i = 1; i < joystickButtons0.length; i++) {
      joystickButtons0[i] = new JoystickButton(joystick0, i);
      joystickButtons1[i] = new JoystickButton(joystick1, i);
      operatorButtons[i] = new JoystickButton(operatorConsole, i);
    }

    for (int i = 1; i < xboxButtons.length; i++) {
      xboxButtons[i] = new JoystickButton(xboxController, i);
    }

    // all subsystems must register with the Command Scheduler in order for their periodic methods
    //  to be invoked
    drivetrain.register();
    storage.register();
    collector.register();
    flywheel.register();
    storage.register();
    elevator.register();
    secondMechanism.register();
    pneumatics.register();

    // Set up the default command for the drivetrain.
    // The joysticks' values map to percentage of the maximum velocities.
    // The velocities may be specified from either the robot's frame of reference or the field's
    //  frame of reference. In the robot's frame of reference, the positive x direction is
    //  forward; the positive y direction, left; position rotation, CCW. In the field frame of
    // reference,
    //  the origin of the field to the lower left corner (i.e., the corner of the field to the
    //  driver's right). Zero degrees is away from the driver and increases in the CCW direction.
    // This is why the left joystick's y axis specifies the velocity in the x direction and the
    //  left joystick's x axis specifies the velocity in the y direction.
    drivetrain.setDefaultCommand(
        new DefaultDriveCommand(
            drivetrain,
            () ->
                -modifyAxis(joystick0.getY()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
            () ->
                -modifyAxis(joystick0.getX()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
            () ->
                -modifyAxis(joystick1.getX())
                    * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    configureButtonBindings();
    configureAutoCommands();

    CommandScheduler.getInstance()
        .onCommandInitialize(
            command ->
                Shuffleboard.addEventMarker(
                    "Command initialized",
                    command
                        .getClass()
                        .getName()
                        .substring(command.getClass().getName().lastIndexOf('.') + 1),
                    EventImportance.kNormal));

    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command ->
                Shuffleboard.addEventMarker(
                    "Command interrupted",
                    command
                        .getClass()
                        .getName()
                        .substring(command.getClass().getName().lastIndexOf('.') + 1),
                    EventImportance.kNormal));

    CommandScheduler.getInstance()
        .onCommandFinish(
            command ->
                Shuffleboard.addEventMarker(
                    "Command finished",
                    command
                        .getClass()
                        .getName()
                        .substring(command.getClass().getName().lastIndexOf('.') + 1),
                    EventImportance.kNormal));
  }

  /**
   * Factory method to create the singleton robot container object.
   *
   * @return the singleton robot container object
   */
  public static RobotContainer getInstance() {
    return robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureDrivetrainButtons();
    configureIntakeButtons();
    configureShooterButtons();
    configureClimberButtons();
  }

  private void configureDrivetrainButtons() {
    // auto aim and shoot while moving
    operatorButtons[JoystickConstants.AUTO_AIM_AND_SHOOT].whenPressed(
        new SequentialCommandGroup(
            new IndexSingleBallCommand(storage),
            new LimelightAlignOnMoveCommand(
                drivetrain,
                flywheel,
                collector,
                () ->
                    -modifyAxis(joystick0.getY())
                        * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () ->
                    -modifyAxis(joystick0.getX())
                        * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () ->
                    -modifyAxis(joystick1.getX())
                        * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND),
            new WaitCommand(0.300),
            createLimelightShootCommandSequence(true /* use gyro */)));

    // field-relative toggle
    joystickButtons0[3].toggleWhenPressed(
        new ConditionalCommand(
            new InstantCommand(() -> drivetrain.disableFieldRelative(), drivetrain),
            new InstantCommand(() -> drivetrain.enableFieldRelative(), drivetrain),
            drivetrain::getFieldRelative));

    // rotate evasively around defending robot
    joystickButtons1[4].whenPressed(
        new InstantCommand(
            () ->
                drivetrain.rotateEvasively(
                    -modifyAxis(joystick0.getY()),
                    -modifyAxis(joystick0.getX()),
                    -modifyAxis(joystick1.getX())),
            drivetrain));
    joystickButtons1[4].whenReleased(
        new InstantCommand(() -> drivetrain.resetCenterGrav(), drivetrain));

    // stop/disable/reset all subsystems
    joystickButtons1[3].whenPressed(
        new ParallelCommandGroup(
            new InstantCommand(() -> flywheel.stopFlywheel(), flywheel),
            new InstantCommand(() -> storage.disableStorage(), storage),
            new InstantCommand(() -> collector.disableCollector(), collector),
            new InstantCommand(() -> drivetrain.disableXstance(), drivetrain)));

    // reset gyro to 0 degrees
    xboxButtons[BUTTON_Y].whenPressed(
        new InstantCommand(() -> drivetrain.zeroGyroscope(), drivetrain));
  }

  private void configureIntakeButtons() {
    // toggle collector
    xboxButtons[JoystickConstants.LEFT_JOYSTICK_BUTTON].toggleWhenPressed(
        new ConditionalCommand(
            new InstantCommand(() -> collector.disableCollector(), collector),
            new InstantCommand(() -> collector.enableCollector(), collector),
            collector::isEnabled));

    // toggle storage
    xboxButtons[JoystickConstants.RIGHT_JOYSTICK_BUTTON].toggleWhenPressed(
        new ConditionalCommand(
            new InstantCommand(() -> storage.disableStorage(), storage),
            new InstantCommand(() -> storage.enableStorage(), storage),
            storage::isStorageEnabled));

    // intake
    joystickButtons1[1].toggleWhenPressed(
        new ConditionalCommand(
            new ParallelCommandGroup(
                new InstantCommand(() -> collector.disableCollector(), collector),
                new InstantCommand(() -> storage.disableStorage(), storage),
                new InstantCommand(() -> flywheel.stopFlywheel(), flywheel)),
            new SequentialCommandGroup(
                new InstantCommand(() -> collector.enableCollector(), collector),
                new SortStorageCommand(storage),
                new InstantCommand(() -> collector.disableCollector(), collector),
                new SetFlywheelVelocityCommand(flywheel, FlywheelConstants.WALL_SHOT_VELOCITY)),
            collector::isEnabled));

    // unjam all
    xboxButtons[BUTTON_X].whenHeld(
        new ParallelCommandGroup(
            new InstantCommand(
                () -> collector.setCollectorPower(CollectorConstants.OUTTAKE_POWER), collector),
            new InstantCommand(
                () -> storage.setStoragePower(StorageConstants.OUTTAKE_POWER), storage),
            new InstantCommand(() -> flywheel.unjamFlywheel(), flywheel)));
    xboxButtons[BUTTON_X].whenReleased(
        new ParallelCommandGroup(
            new InstantCommand(() -> collector.setCollectorPower(0), collector),
            new InstantCommand(() -> storage.setStoragePower(0), storage),
            new InstantCommand(() -> flywheel.stopFlywheel(), flywheel)));

    // unjam collector
    xboxButtons[BUTTON_B].whenHeld(
        new InstantCommand(
            () -> collector.setCollectorPower(CollectorConstants.OUTTAKE_POWER), collector));
    xboxButtons[BUTTON_B].whenReleased(
        new InstantCommand(() -> collector.setCollectorPower(0), collector));

    joystickButtons0[2].whenPressed(
        new ParallelRaceGroup(
            new SortStorageCommand(storage),
            new VisionBoxCollectBallCommand(visionBox, drivetrain, collector, storage)));
  }

  private void configureShooterButtons() {

    // enable/disable limelight aiming
    operatorButtons[JoystickConstants.LIMELIGHT_AIM_TOGGLE].toggleWhenPressed(
        new ConditionalCommand(
            new InstantCommand(() -> drivetrain.disableLimelightAim(), drivetrain),
            new InstantCommand(() -> drivetrain.enableLimelightAim(), drivetrain),
            drivetrain::isLimelightAimEnabled));

    // preset field wall shot
    operatorButtons[JoystickConstants.FIELD_WALL].whenPressed(
        createShootCommandSequence(FlywheelConstants.WALL_SHOT_VELOCITY));

    // preset launchpad shot
    operatorButtons[JoystickConstants.LAUNCHPAD].whenPressed(
        createShootCommandSequence(FlywheelConstants.LAUNCH_PAD_VELOCITY));

    // shoot slow
    operatorButtons[JoystickConstants.SHOOT_SLOW].whenPressed(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> collector.disableCollector(), collector),
                new SetFlywheelVelocityCommand(flywheel, FlywheelConstants.SHOOT_SLOW_VELOCITY)),
            new InstantCommand(() -> storage.enableStorage(), storage),
            new WaitForShotCommand(storage, flywheel, drivetrain)));

    // limelight shot
    joystickButtons0[1].whenPressed(
        new SequentialCommandGroup(
            new ParallelCommandGroup(new IndexSingleBallCommand(storage), new WaitCommand(0.300)),
            createLimelightShootCommandSequence(true /* use gyro */)));
  }

  private void configureClimberButtons() {

    // configure climb to 4 (traverse) rung climb sequence
    operatorButtons[8].whenPressed(
        new SequentialCommandGroup(
            new RetractClimberFullCommand(elevator),
            new InstantCommand(() -> secondMechanism.moveSecondaryArmOut(), secondMechanism),
            new WaitCommand(0.5), // wait for secondary arm to be positioned
            new ReachToNextRungWithPitchCommand(elevator, secondMechanism),
            new WaitCommand(
                ElevatorConstants
                    .RETRACT_DELAY_AFTER_EXTENSION_UNDER_RUNG), // wait for secondary arm to be
            // positioned
            new RetractClimberMinimumCommand(
                ElevatorConstants.LATCH_TRAVERSE_RUNG_ENCODER_HEIGHT, elevator)));

    // configure climb to 3 (high) rung climb sequence
    operatorButtons[7].whenPressed(
        new SequentialCommandGroup(
            new RetractClimberFullCommand(elevator),
            new InstantCommand(() -> secondMechanism.moveSecondaryArmOut(), secondMechanism),
            new WaitCommand(0.5), // wait for secondary arm to be positioned
            new ReachToNextRungWithPitchCommand(elevator, secondMechanism),
            new WaitCommand(
                ElevatorConstants
                    .RETRACT_DELAY_AFTER_EXTENSION_UNDER_RUNG), // wait for secondary arm to be
            // positioned
            new RetractClimberMinimumCommand(
                ElevatorConstants.LATCH_HIGH_RUNG_ENCODER_HEIGHT, elevator)));

    // configure climb to 1/2 (low/mid) rung climb sequence
    operatorButtons[1].whenPressed(
        new SequentialCommandGroup(
            new RetractClimberFullCommand(elevator),
            new InstantCommand(() -> secondMechanism.moveSecondaryArmOut(), secondMechanism)));

    // configure raise elevator before starting climb to 1 (low) rung
    operatorButtons[11].whenPressed(new ExtendClimberToLowRungCommand(elevator));

    // configure raise elevator before starting climb
    operatorButtons[2].whenPressed(new ExtendClimberToMidRungCommand(elevator));

    // retract climber
    xboxButtons[BUTTON_A].whenPressed(new RetractClimberFullCommand(elevator));

    // manually extend climber
    xboxButtons[JoystickConstants.BUTTON_RB].whenPressed(
        new InstantCommand(
            () -> elevator.setElevatorMotorPower(ElevatorConstants.DEFAULT_MOTOR_POWER), elevator));
    xboxButtons[JoystickConstants.BUTTON_RB].whenReleased(
        new InstantCommand(() -> elevator.setElevatorMotorPower(0), elevator));

    // manually retract climber
    xboxButtons[JoystickConstants.BUTTON_LB].whenPressed(
        new InstantCommand(
            () -> elevator.setElevatorMotorPower(-1 * ElevatorConstants.DEFAULT_MOTOR_POWER),
            elevator));
    xboxButtons[JoystickConstants.BUTTON_LB].whenReleased(
        new InstantCommand(() -> elevator.setElevatorMotorPower(0), elevator));

    // pause elevator
    xboxButtons[JoystickConstants.BUTTON_START].whenPressed(
        new InstantCommand(
            () -> elevator.elevatorPause(xboxButtons[JoystickConstants.BUTTON_BACK].get()),
            elevator));

    // enable elevator control
    operatorButtons[12].toggleWhenPressed(
        new ConditionalCommand(
            new InstantCommand(() -> elevator.disableElevatorControl(), elevator),
            new InstantCommand(() -> elevator.enableElevatorControl(), elevator),
            elevator::isElevatorControlEnabled));

    // manually toggle secondary arms
    operatorButtons[JoystickConstants.SECONDARY].toggleWhenPressed(
        new ConditionalCommand(
            new InstantCommand(() -> secondMechanism.moveSecondaryArmOut()),
            new InstantCommand(() -> secondMechanism.moveSecondaryArmIn()),
            secondMechanism::isIn));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  /** Start teleop in a known state (i.e., storage, flywheel, collector stopped/disabled) */
  public void teleopInit() {
    storage.disableStorage();
    flywheel.stopFlywheel();
    collector.disableCollector();
    visionBox.updateBallColorConstants();
  }

  public void disabledPeriodic() {
    // By invoking the stop method, each swerve module's set method will be invoked when the robot
    // is disabled.
    // This is important since it provides an opportunity for the absolute angle to be properly
    // read and the seed angle to be fixed before starting auto. As long as the robot is running for
    // at most 10 seconds, the angles will be correct.
    drivetrain.stop();
  }

  private void configureAutoCommands() {
    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            AutoConstants.P_THETA_CONTROLLER,
            0,
            0,
            AutoConstants.THETA_CONTROLLER_CONSTRAINTS,
            LOOP_PERIOD_SECS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PathPlannerTrajectory autoBlue01Path =
        PathPlanner.loadPath(
            "Blue0(1)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    PathPlannerTrajectory autoBlue02Path =
        PathPlanner.loadPath(
            "Blue0(2)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command autoOneBall =
        new SequentialCommandGroup(
            new FollowPath(autoBlue01Path, thetaController, drivetrain, true),
            createAutoShootCommandSequence(FlywheelConstants.WALL_SHOT_VELOCITY, 2),
            new FollowPath(autoBlue02Path, thetaController, drivetrain, false));

    PathPlannerTrajectory autoBlue11Path =
        PathPlanner.loadPath(
            "Blue1(1)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    PathPlannerTrajectory autoBlue12Path =
        PathPlanner.loadPath(
            "Blue1(2)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command autoTwoBallSteal =
        new SequentialCommandGroup(
            new InstantCommand(() -> collector.enableCollector(), collector),
            new FollowPath(autoBlue11Path, thetaController, drivetrain, true),
            new WaitCommand(5.0),
            createAutoShootCommandSequence(FlywheelConstants.WALL_SHOT_VELOCITY, 2),
            new ParallelDeadlineGroup(
                new FollowPath(autoBlue12Path, thetaController, drivetrain, false),
                new SortStorageCommand(storage)),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new InstantCommand(() -> collector.disableCollector(), collector),
                    new SetFlywheelVelocityCommand(
                        flywheel, FlywheelConstants.SHOOT_STEAL_VELOCITY)),
                new InstantCommand(() -> storage.enableStorage(), storage),
                new WaitCommand(15)));

    Command autoTwoBall =
        new SequentialCommandGroup(
            new InstantCommand(() -> collector.enableCollector(), collector),
            new FollowPath(autoBlue11Path, thetaController, drivetrain, true),
            new WaitCommand(5.0),
            createAutoShootCommandSequence(FlywheelConstants.WALL_SHOT_VELOCITY, 15));

    PathPlannerTrajectory autoBlue2Path =
        PathPlanner.loadPath(
            "Blue2(1)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command autoTwoBallAlt =
        new SequentialCommandGroup(
            new InstantCommand(() -> collector.enableCollector(), collector),
            new FollowPath(autoBlue2Path, thetaController, drivetrain, true),
            new InstantCommand(() -> collector.disableCollector(), collector),
            createAutoShootCommandSequence(FlywheelConstants.WALL_SHOT_VELOCITY, 15));

    // 5 ball auto (shoot 2; shoot 3 (with bowling))
    PathPlannerTrajectory autoBlue31Path =
        PathPlanner.loadPath(
            "Blue3(1)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    PathPlannerTrajectory autoBlue32Path =
        PathPlanner.loadPath(
            "Blue3(2)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    PathPlannerTrajectory autoBlue33Path =
        PathPlanner.loadPath(
            "Blue3(3)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command autoFiveBall =
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> collector.enableCollector(), collector),
                new InstantCommand(
                    () -> flywheel.setVelocity(FlywheelConstants.NEAR_WALL_SHOT_VELOCITY),
                    flywheel)),
            new WaitCommand(0.5),
            new FollowPath(autoBlue31Path, thetaController, drivetrain, true),
            createAutoShootNoAimCommandSequence(FlywheelConstants.NEAR_WALL_SHOT_VELOCITY, 2),
            new ParallelDeadlineGroup(
                new FollowPath(autoBlue32Path, thetaController, drivetrain, false),
                new SortStorageCommand(storage)),
            new ParallelCommandGroup(
                new InstantCommand(
                    () -> flywheel.setVelocity(FlywheelConstants.LAUNCH_PAD_VELOCITY), flywheel),
                new FollowPath(autoBlue33Path, thetaController, drivetrain, false)),
            limelightCreateAutoShootCommandSequence(15));

    // 5-ball auto (shoot 2, shoot 1, shoot 2 (no bowling))
    PathPlannerTrajectory autoBlue41Path =
        PathPlanner.loadPath(
            "Blue4(1)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    PathPlannerTrajectory autoBlue42Path =
        PathPlanner.loadPath(
            "Blue4(2)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    PathPlannerTrajectory autoBlue43Path =
        PathPlanner.loadPath(
            "Blue4(3)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    PathPlannerTrajectory autoBlue44Path =
        PathPlanner.loadPath(
            "Blue4(4)",
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command autoFiveBallAlt =
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> collector.enableCollector(), collector),
                new InstantCommand(
                    () -> flywheel.setVelocity(FlywheelConstants.WALL_SHOT_VELOCITY), flywheel)),
            new WaitCommand(0.5),
            new FollowPath(autoBlue41Path, thetaController, drivetrain, true),
            limelightCreateAutoShootCommandSequence(2),
            new ParallelDeadlineGroup(
                new FollowPath(autoBlue42Path, thetaController, drivetrain, false),
                new SortStorageCommand(storage)),
            limelightCreateAutoShootCommandSequence(2),
            new FollowPath(autoBlue43Path, thetaController, drivetrain, false),
            new InstantCommand(() -> drivetrain.stop()),
            new ParallelDeadlineGroup(new WaitCommand(2), new SortStorageCommand(storage)),
            new ParallelCommandGroup(
                new InstantCommand(
                    () -> flywheel.setVelocity(FlywheelConstants.LAUNCH_PAD_VELOCITY), flywheel),
                new FollowPath(autoBlue44Path, thetaController, drivetrain, false)),
            limelightCreateAutoShootCommandSequence(5));

    ShuffleboardTab tab = Shuffleboard.getTab("MAIN");
    chooser.addOption("1 Ball", autoOneBall);
    chooser.addOption("2 Ball & Steal", autoTwoBallSteal);
    chooser.addOption("2 Ball", autoTwoBall);
    chooser.addOption("Alt 2 Ball", autoTwoBallAlt);
    chooser.addOption("Main 5 Ball", autoFiveBall);
    chooser.addOption("Alt 5 Ball", autoFiveBallAlt);
    tab.add("Auto Mode", chooser);
  }

  private Command createShootCommandSequence(int shotVelocity) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new InstantCommand(() -> collector.disableCollector(), collector),
            new SetFlywheelVelocityCommand(flywheel, shotVelocity),
            new SequentialCommandGroup(
                new LimelightAlignWithGyroCommand(drivetrain),
                new InstantCommand(() -> drivetrain.enableXstance(), drivetrain))),
        new InstantCommand(() -> storage.enableStorage(), storage),
        new WaitForShotCommand(storage, flywheel, drivetrain));
  }

  private Command createLimelightShootCommandSequence(boolean useGyro) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new InstantCommand(() -> collector.disableCollector(), collector),
            new LimelightSetFlywheelVelocityCommand(flywheel, drivetrain),
            new SequentialCommandGroup(
                (useGyro
                    ? new LimelightAlignWithGyroCommand(drivetrain)
                    : new LimelightAlignToTargetCommand(drivetrain)),
                new InstantCommand(() -> drivetrain.enableXstance(), drivetrain))),
        new InstantCommand(() -> storage.enableStorage(), storage),
        new WaitForShotCommand(storage, flywheel, drivetrain));
  }

  private Command createAutoShootCommandSequence(int shotVelocity, double shotDelay) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new SetFlywheelVelocityCommand(flywheel, shotVelocity),
            new LimelightAlignWithGyroCommand(drivetrain)),
        new InstantCommand(() -> storage.enableStorage(), storage),
        new WaitCommand(shotDelay),
        new ParallelCommandGroup(
            new InstantCommand(() -> flywheel.stopFlywheel(), flywheel),
            new InstantCommand(() -> storage.disableStorage(), storage)));
  }

  private Command createAutoShootNoAimCommandSequence(int shotVelocity, double shotDelay) {
    return new SequentialCommandGroup(
        new SetFlywheelVelocityCommand(flywheel, shotVelocity),
        new InstantCommand(() -> storage.enableStorage(), storage),
        new WaitCommand(shotDelay),
        new ParallelCommandGroup(
            new InstantCommand(() -> flywheel.stopFlywheel(), flywheel),
            new InstantCommand(() -> storage.disableStorage(), storage)));
  }

  private Command limelightCreateAutoShootCommandSequence(double shotDelay) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new LimelightSetFlywheelVelocityCommand(flywheel, drivetrain),
            new LimelightAlignWithGyroCommand(drivetrain)),
        new InstantCommand(() -> storage.enableStorage(), storage),
        new WaitCommand(shotDelay),
        new ParallelCommandGroup(
            new InstantCommand(() -> flywheel.stopFlywheel(), flywheel),
            new InstantCommand(() -> storage.disableStorage(), storage)));
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  /**
   * Squares the specified value, while preserving the sign. This method is used on all joystick
   * inputs. This is useful as a non-linear range is more natural for the driver.
   *
   * @param value
   * @return
   */
  public static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
