// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public class xboxConstants{

   public static final int xboxControllerPort = 0;

  }
    public class MotorConstants{
      public static final int motorID = 10;
      public static final int proxSensorPort = 4;

      public static final double motorSpeed = 0.2;

      public static final double motorP = 0.02;
      public static final double motorI = 0;
      public static final double motorD = 0;
      public static final int motorRevolutions = 30;
    }
  }

