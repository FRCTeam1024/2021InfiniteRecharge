
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.BallFeed;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class RunBothFeeders extends ParallelCommandGroup {
  /**
   * Creates a new RunBothFeeders.
   */
  public RunBothFeeders(BallFeed ballFeed) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
    super(new RunShooterFeed(ballFeed, -1.0), new RunBallFeed(ballFeed, 0.50));
  }
}
