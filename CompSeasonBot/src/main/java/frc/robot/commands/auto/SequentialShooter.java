package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.RunBothFeeders;
import frc.robot.subsystems.BallFeed;
import frc.robot.subsystems.Shooter;

public class SequentialShooter extends SequentialCommandGroup {
  
   // Creates a new SequentialShooter.
   BallFeed ballFeed;
   Shooter shooter;
   
  public SequentialShooter(Shooter shooter, BallFeed ballFeed) {
     // Add your commands in the super() call, e.g.
     super(new WaitUntilCommand(shooter::isAtMaxRPM),
          new RunBothFeeders(ballFeed).withInterrupt(shooter::isNotAtMaxRPM),
          new WaitUntilCommand(shooter::isAtMaxRPM),
          new RunBothFeeders(ballFeed).withInterrupt(shooter::isNotAtMaxRPM),
          new WaitUntilCommand(shooter::isAtMaxRPM),
          new RunBothFeeders(ballFeed).withInterrupt(shooter::isNotAtMaxRPM)
          );
          System.out.println("running SequentialShooter");
  }
}
