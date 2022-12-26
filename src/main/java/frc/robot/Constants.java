package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double LOOP_PERIOD_SECS = 0.02;
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  public static Mode getMode() {
    return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
  }

  /* DRIVETRAIN CONSTANTS */

  public static final class DrivetrainConstants {

    private DrivetrainConstants() {
      throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 8;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(118.0371);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 13;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 14;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(102.9968);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 9;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(172.7051);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 16;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 15;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 17;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(40.3335);

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * <p>Should be measured from center to center.
     */
    public static final double TRACKWIDTH_METERS = 0.5715; // 22.5 inches

    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * <p>Should be measured from center to center.
     */
    public static final double WHEELBASE_METERS = 0.5969; // 23.5 inches

    public static final double ROBOT_WIDTH_WITH_BUMPERS = 0.89; // meters
    public static final double ROBOT_LENGTH_WITH_BUMPERS = 0.91; // meters
    public static final double EVASIVE_ROTATION_COG_SHIFT_MAGNITUDE =
        0.707; // a bit beyond the bumper permimeter (meters)
    public static final double COG_OFFSET = 45;

    /**
     * The maximum voltage that will be delivered to the drive motors.
     *
     * <p>This can be reduced to cap the robot's maximum speed. Typically, this is useful during
     * initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 13.0;

    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    // By default this value is setup for a Mk3 standard module using Falcon500s to drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI

    /**
     * The maximum velocity of the robot in meters per second.
     *
     * <p>This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND =
        6380.0
            / 60.0
            * SdsModuleConfigurations.MK4_L2.getDriveReduction()
            * SdsModuleConfigurations.MK4_L2.getWheelDiameter()
            * Math.PI;

    /**
     * The maximum angular velocity of the robot in radians per second.
     *
     * <p>This is a measure of how fast the robot can rotate in place.
     */

    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
        MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

    public static final int PIGEON_ID = 18;
    public static final int TIMEOUT_MS = 30;

    /* Limelight */
    public static final String LIMELIGHT_NETWORK_TABLE_NAME = "limelight";
    public static final double LIMELIGHT_F = 0.1;
    public static final double LIMELIGHT_P = 0.2;
    public static final double LIMELIGHT_I = 0.50;
    public static final double LIMELIGHT_ALIGNMENT_TOLERANCE = 1.0;
    public static final double LIMELIGHT_LAUNCHPAD_ALIGNMENT_TOLERANCE = .6;
    public static final double LIMELIGHT_AIM_TOLERANCE = 3; // inches
    public static final int AIM_SETPOINT_COUNT = 2;
    public static final double LIMELIGHT_SLOPE = 24;
    public static final double LIMELIGHT_Y_COMPONENT = 4596.34;

    // FIXME: determine the latency by filming a phone timer and the camera video of same phone
    // timer; check Pose Estimation document for details
    public static final double LIMELIGHT_LATENCY = 0.05;

    /* Rev Hubs */
    public static final int POWER_DISTRIBUTION_HUB_ID = 21;
  }

  public static final class PneumaticsConstants {
    private PneumaticsConstants() {
      throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    public static final int PNEUMATICS_HUB_ID = 20;
    public static final int FLOW_SENSOR_CHANNEL = 0;
    public static final int PRESSURE_SENSOR_CHANNEL = 1;
  }

  public static final class CollectorConstants {

    private CollectorConstants() {
      throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    public static final double OUTTAKE_POWER = -0.7;
    public static final int COLLECTOR_MOTOR_ID = 5;
    public static final int PEUNAMATICS_HUB_CAN_ID = 20;
    public static final int COLLECTOR_SOLENOID_CHANNEL = 0;
    public static final double COLLECTOR_DEFAULT_POWER = 0.9;
    public static final int TIMEOUT_MS = 30;
  }

  public static final class AutoConstants {

    private AutoConstants() {
      throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    // from sysId tool
    public static final double S_VOLTS = 0.55493;
    public static final double V_VOLT_SECONDS_PER_METER = 2.3014;
    public static final double A_VOLT_SECONDS_SQUARED_PER_METER = 0.12872;

    public static final double MAX_SPEED_METERS_PER_SECOND = 3;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2.0 * Math.PI;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = 2.0 * Math.PI;

    public static final double PX_CONTROLLER = 2.2956; // from sysId tool
    public static final double PY_CONTROLLER = 2.2956; // from sysId tool
    public static final double P_THETA_CONTROLLER = 4.9;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
  }

  public static class FlywheelConstants {

    private FlywheelConstants() {
      throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    public static final int SLOT_INDEX = 0;
    public static final int PID_LOOP_INDEX = 0;
    public static final int TIMEOUT_MS = 30;
    public static final int VELOCITY_TOLERANCE = 300;
    public static final int LEFT_FLYWHEELMOTOR_CANID = 1;
    public static final int RIGHT_FLYWHEELMOTOR_CANID = 2;
    public static final double VELOCITY_PID_P = 0.18;
    public static final double VELOCITY_PID_I = 0.0;
    public static final double VELOCITY_PID_D = 0.0;
    public static final double VELOCITY_PID_F = 0.0513;
    public static final double VELOCITY_PID_I_ZONE = 0.0;
    public static final double VELOCITY_PID_PEAK_OUTPUT = 1.0;
    public static final int MAX_FLYWHEEL_VELOCITY = 18650; // ticks per 100 ms
    public static final int NEAR_WALL_SHOT_VELOCITY = 7000; // ticks per 100 ms
    public static final int WALL_SHOT_VELOCITY = 7682; // ticks per 100 ms
    public static final int FENDER_SHOT_VELOCITY = 7799; // ticks per 100 ms
    public static final int LAUNCH_PAD_VELOCITY = 8682; // ticks per 100 ms
    public static final int SHOOT_SLOW_VELOCITY = 4000; // ticks per 100 ms
    public static final int SHOOT_STEAL_VELOCITY = 5500; // ticks per 100 ms
    public static final double REVERSE_POWER = -0.2;
    public static final int SETPOINTCOUNT = 5;
  }

  public static class LimelightConstants {

    private LimelightConstants() {
      throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    public static final double HUB_H = 104; // inches
    public static final double ROBOT_H = 21.25; // inches
    public static final double GRAV_CONST_FT = -32.17519788;
    public static final double FLYWHEEL_RADIUS_INCHES = 2; // inches
    public static final double VELOCITY_MULTIPLIER = 2;
    public static final double TICKS_PER_ROTATION = 2048;
    public static final int LIMELIGHT_ANGLE_OFFSET = -2; // degrees
    public static final int LIMELIGHT_MOUNT_ANGLE = 45; // degrees
    public static final int D2_D1_OFFSET_IN = 24; // inches
    public static final int H2_H1_OFFSET_IN = -24; // inches
    public static final int DISTANCE_TOLERANCE = 12; // inches
    // 203" from center of hub to center of lauchpad
    //  26" from edge of hub to center of hub
    //  3" from center of launch pad to bumpers
    //  18" from bumpers to center of robot
    //  7.5" from center of robot to Limeligtht
    public static final int HUB_LAUNCHPAD_DISTANCE = 149; // inches
    public static final int EDGE_TO_CENTER_HUB_DISTANCE = 26 + 8; // inches
    public static final int HUB_WALL_DISTANCE = 130; // inches
    public static final int AUTO_SHOT_HUB_FAR_DISTANCE = 124; // inches
    public static final int AUTO_SHOT_HUB_CLOSE_DISTANCE = 68; // inches
  }

  public static class VisionBoxConstants {

    private VisionBoxConstants() {
      throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    public static final double CAMERA_HEIGHT_METERS =
        .75184; // TODO: remeasure these at competition
    public static final double CAMERA_ANGLE_DEGREES = 32.5;
    public static final double X_KP = -0.8; // stolen from autoconstants, may be horribly wrong
    public static final double X_KI = 0;
    public static final double X_KD = 0;
    public static final double X_MAX_ACCELERATION = 10; // TODO verify, adapted from AutoConstants
    public static final double X_MAX_VELOCITY = 5; // TODO verify, adapted from AutoConstants
    public static final double Y_KP = -0.8;
    public static final double Y_KI = 0;
    public static final double Y_KD = 0;
    public static final double Y_MAX_ACCELERATION = 15; // TODO verify, adapted from AutoConstants
    public static final double Y_MAX_VELOCITY = 3; // TODO verify, adapted from AutoConstants
    public static final double ROTATIONAL_KP =
        -6; // stolen from drivetrain constants limelight PID but converted from deg to rad, may be
    // horribly wrong
    public static final double ROTATIONAL_KI = 0;
    public static final double ROTATIONAL_KD = 0;
    public static final double ROTATIONAL_MAX_ACCELERATION =
        2.0 * Math.PI; // TODO verify, adapted from AutoConstants
    public static final double ROTATIONAL_MAX_VELOCITY =
        Math.PI; // TODO verify, adapted from DrivetrainSubsystem.aim()
    public static final double AIM_TOLERANCE_DEGREES = 5;
    public static final double AIM_TOLERANCE_METERS = .33;
    public static final double MINIMUM_UNAIMED_DISTANCE_METERS = 1; // about 3ft
    public static final double OVERSHOOT_DISTANCE_METERS = .6; // about 2ft
    public static final double MAX_DISPLACEMENT_PER_TICK_METERS =
        .036; // this is way too high. should be (relative max ball velocity * .02s)
    public static final double AT_BALL_THRESHOLD_METERS = .1;
  }

  public static final class StorageConstants {

    private StorageConstants() {
      throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    public static final double OUTTAKE_POWER = -.8;
    public static final int SHOOTER_SENSOR = 1;
    public static final int COLLECTOR_SENSOR = 0;
    public static final int STORAGE_MOTOR_ID = 4;
    public static final double STORAGE_DEFAULT_POWER = 0.7;
    public static final int STORAGE_CAMERA_PORT = 0;
    public static final int TIMEOUT_MS = 30;
    public static final int WAIT_FOR_SHOT_DELAY = 10;
    public static final int INDEXING_FORWARD_DELAY = 16;
    public static final int INDEXING_BACKWARD_DURATION = 3;
  }

  public static class ElevatorConstants {

    private ElevatorConstants() {
      throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    public static final double MIN_ELEVATOR_ENCODER_HEIGHT = 0;
    public static final double TRANSFER_TO_SECONDARY_HEIGHT = 25150;
    public static final double LOW_RUNG_HEIGHT = 150786;
    public static final double REACH_JUST_BEFORE_NEXT_RUNG = 185227;
    public static final double MID_RUNG_HEIGHT = 253338;

    public static final double NEXT_RUNG_HEIGHT = 227499;
    public static final double LATCH_HIGH_RUNG_ENCODER_HEIGHT = 238151;
    public static final double LATCH_TRAVERSE_RUNG_ENCODER_HEIGHT = 171592;
    public static final double REACH_TO_NEXT_RUNG_HEIGHT = 265674;
    public static final double MAX_ELEVATOR_HEIGHT = 272631;
    public static final double TICKS_PER_INCH = 8874.266;

    public static final double RETRACT_DELAY_AFTER_EXTENSION_UNDER_RUNG = 0.040;

    public static final int ELEVATOR_POSITION_TOLERANCE = 1000;
    public static final double ARBITRARY_FEED_FORWARD_EXTEND = .02;
    public static final double ARBITRARY_FEED_FORWARD_RETRACT = -0.07;
    public static final double DEFAULT_MOTOR_POWER = 0.5;

    public static final int SLOT_INDEX = 0;
    public static final int PID_LOOP_INDEX = 0;
    public static final int TIMEOUT_MS = 30;
    public static final double POSITION_PID_P = 0.4;
    public static final double POSITION_PID_I = 0.0;
    public static final double POSITION_PID_D = 0.0;
    public static final double POSITION_PID_F = 0.0;
    public static final double POSITION_PID_I_ZONE = 0.0;
    public static final double POSITION_PID_PEAK_OUTPUT = 1.0;
    public static final double SLOW_PEAK_OUTPUT = 0.15;
    public static final double MAX_ELEVATOR_VELOCITY = 20000; // theoretical maximum 21305
    public static final double ELEVATOR_ACCELERATION = MAX_ELEVATOR_VELOCITY * 10;
    public static final int SCURVE_STRENGTH = 0;

    public static final int SAMPLE_WINDOW_WIDTH = 6;
    public static final double EPSILON = 0.001;

    // CAN ID
    public static final int PIGEON_ID = 18;
    public static final int LEFT_ELEVATOR_MOTOR_CAN_ID = 22;
    public static final int RIGHT_ELEVATOR_MOTOR_CAN_ID = 19;
    public static final int CLIMBER_CAMERA_PORT = 0;
  }

  public class SecondaryArmConstants {

    private SecondaryArmConstants() {
      throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    public static final int PNEUMATIC_HUB_CAN_ID = 20;
    public static final int PNEUMATIC_CHANNEL = 1;
  }

  public static final class JoystickConstants {

    private JoystickConstants() {
      throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    public static final int CLIMBER_UP = 2;
    public static final int CLIMB_2 = 1;
    public static final int CLIMB_3 = 7;
    public static final int CLIMB_4 = 8;
    public static final int LIMELIGHT_AIM_TOGGLE = 9;
    public static final int FIELD_WALL = 6;
    public static final int LAUNCHPAD = 5;
    public static final int SECONDARY = 4;
    public static final int AUTO_AIM_AND_SHOOT = 3;
    public static final int SHOOT_SLOW = 10;
    public static final int UNASSIGNED = 11;
    public static final int CLIMB_CAM = 12;

    public static final int BUTTON_A = 1;
    public static final int BUTTON_B = 2;
    public static final int BUTTON_X = 3;
    public static final int BUTTON_Y = 4;
    public static final int BUTTON_LB = 5;
    public static final int BUTTON_RB = 6;
    public static final int BUTTON_BACK = 7;
    public static final int BUTTON_START = 8;
    public static final int LEFT_JOYSTICK_BUTTON = 9;
    public static final int RIGHT_JOYSTICK_BUTTON = 10;
  }

  public enum Mode {
    REAL,
    REPLAY,
    SIM
  }
}
