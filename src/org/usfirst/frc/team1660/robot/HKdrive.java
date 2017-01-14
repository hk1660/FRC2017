/* Modified version of FRC's WPI RoboDrive class */

package org.usfirst.frc.team1660.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
//import edu.wpi.first.wpilibj.SuppressWarnings;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tInstances;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.hal.HAL;
import static java.util.Objects.requireNonNull;
/**
 * Utility class for handling Robot drive based on a definition of the motor configuration. The
 * robot drive class handles basic driving for a robot. Currently, 2 and 4 motor tank and mecanum
 * drive trains are supported. In the future other drive types like swerve might be implemented.
 * Motor channel numbers are supplied on creation of the class. Those are used for either the drive
 * function (intended for hand created drive code, such as autonomous) or with the Tank/Arcade
 * functions intended to be used for Operator Control driving.
 */

public class HKdrive extends RobotDrive {
public HKdrive(SpeedController frontLeftMotor, SpeedController rearLeftMotor,
               SpeedController frontRightMotor, SpeedController rearRightMotor) {
  super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
}

/**
 * The location of a motor on the robot for the purpose of driving.
 */
/*public enum MotorType {
  kFrontLeft(0), kFrontRight(1), kRearLeft(2), kRearRight(3);

  @SuppressWarnings("MemberName")
  public final int value;

  private MotorType(int value) {
    this.value = value;
  }
}*/

}

