// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Flywheel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  /** Creates a new Shoot. */
  Flywheel shoot;
  Feeder feed;
  int time = 5000;
  int timer = 0;
  boolean stopCheck = false; 
  public Shoot(Flywheel m_shoot, Feeder m_feed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shoot = m_shoot;
    this.feed = m_feed;
    addRequirements(shoot, feed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer < time) {
      feed.runFeederSD();
      shoot.runFlywheelCommandSD();
      timer++;
    } else {
      stopCheck = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopCheck;
  }
}

