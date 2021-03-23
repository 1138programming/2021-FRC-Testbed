package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

public final class Constants {
  //these are limits you can change!!!
  public static final double kMaxSpeed = Units.feetToMeters(13.6)/100; // 20 feet per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  public static double feildCalibration = 0;

  //this is where you put the angle offsets you got from the smart dashboard

  public static double frontLeftOffset = 0;
  public static double frontRightOffset = 0;
  public static double backLeftOffset = 0;
  public static double backRightOffset = 0;


  //put your can Id's here!
  public static final int frontLeftDriveId = 8; 
  public static final int frontLeftCANifierId = 9; 
  public static final int frontLeftSteerId = 2;
  //put your can Id's here!
  public static final int frontRightDriveId = 7; 
  public static final int frontRightCANifierId = 11; 
  public static final int frontRightSteerId = 5; 
  //put your can Id's here!
  public static final int backLeftDriveId = 6; 
  public static final int backLeftCANifierId = 10; 
  public static final int backLeftSteerId = 3;
  //put your can Id's here!

  public static final int backRightDriveId = 1; 
  public static final int backRightCANifierId = 12; 
  public static final int backRightSteerId = 4; 
}