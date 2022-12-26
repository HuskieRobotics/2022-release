package frc.robot.util;

import com.ctre.phoenix.ErrorCode;
import frc.robot.util.Alert.AlertType;

public class TalonUtil {

  private TalonUtil() {}

  /*
   * checks the specified error code for issues
   *
   * @param errorCode error code
   * @param message   message to print if error happens
   */
  public static void checkError(ErrorCode errorCode, String message) {
    if (errorCode != ErrorCode.OK) {
      Alert talonAlert = new Alert(message + errorCode, AlertType.ERROR);
      talonAlert.set(true);
    }
  }
}
