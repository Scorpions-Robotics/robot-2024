
package frc.robot;

public final class Constants {

  public static class ports {
    public static final int intake_motor_1 = 0;
    public static final int intake_motor_2 = 0;
    public static final int shooter_motor_1 = 0;
    public static final int shooter_motor_2 = 0;

  }

  public static class controller {
    public static final int controller = 0;
  }

/*
   ___           __ _                    _   _            _____     _    _     
 / __|___ _ _  / _(_)__ _ _  _ _ _ __ _| |_(_)___ _ _   |_   _|_ _| |__| |___ 
| (__/ _ \ ' \|  _| / _` | || | '_/ _` |  _| / _ \ ' \    | |/ _` | '_ \ / -_)
 \___\___/_||_|_| |_\__, |\_,_|_| \__,_|\__|_\___/_||_|   |_|\__,_|_.__/_\___|
                    |___/                                                     
                  
 */

  public static class values {

    public static class intake {
    public static final double GetNoteValue = 1;
    public static final double PidIntakeTolerance = 5;
    public static final double PidIntakeKP = 1;
    public static final double PidIntakeKI = 0;
    public static final double PidIntakeKD = 0;
}



public static class shooter {
  public static final double AngleMotorOpenLoopRampRate = 0.5;
  public static final double PidShooterAngleTolerance = 5;
  public static final double PidShooterAngleKP = 1;
  public static final double PidShooterAngleKI = 0;
  public static final double PidShooterAngleKD = 0;

  public static final double PidShooterShootKP = 1;
  public static final double PidShooterShootKI = 0;
  public static final double PidShooterShootKD = 0;
  public static final double PidShooterRPMTolerance = 5;

}


public static class positions {

  public static final double FeedingPositionIntake = 1;
  public static final double FeedingPositionShooter = 1;

}




public static class feeder {

  public static final double FeederForwardSpeed = 0.7;
  public static final double FeederBackwardSpeed = -0.7;

}






  }

}
