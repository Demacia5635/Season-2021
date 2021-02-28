// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.util.HashMap;
import java.util.Scanner;

import org.ejml.equation.Function;

import java.io.File;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TurnToPos.TurnType;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Shooting;

public class AutoShoot extends CommandBase {

  private Shooting shooting;
  private Chassis chassis;
  private SequentialCommandGroup command;

  private boolean firstTime;
  private int cycle;

  private HashMap<Integer, double[]> dictionary;

  public AutoShoot(Shooting shooting, Chassis chassis) {
    this.shooting = shooting;
    this.chassis = chassis;
    dictionary = new HashMap<Integer, double[]>();
    firstTime = true;
    cycle = 0;
    int distance = -1;
    double velocity = -1, angle = -1;
    try {
      Scanner scanner = new Scanner(new File(".\\Utils\\Shooting.json"));
      while (scanner.hasNextLine()) {
        String str = scanner.nextLine();
        String value;
        if (str.contains(":") && !firstTime) {
          value = str.split(":", 2)[1].split("\"", 3)[1];
          if (cycle == 0) {
            distance = Integer.parseInt(value);
            cycle++;
          } else if (cycle == 1) {
            velocity = Double.parseDouble(value);
            cycle++;
          } else {
            angle = Double.parseDouble(value);
            cycle = 0;
            dictionary.put(distance, new double[] { velocity, angle });
          }
        } else if (firstTime && str.contains(":")) firstTime = false;
      }
      scanner.close();
    } catch (IOException e) {
      System.out.println("Couldn't access file");
    }
  }

  @Override
  public void initialize() {
    command = new TurnToPos(chassis, TurnType.Passive, this::getVisionAngle)
        .andThen(new Shoot(shooting, this::getVel, this::getAngle)
            .alongWith(new TurnToPos(chassis, TurnType.Active, this::getVisionAngle)));
    command.schedule();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    command.cancel();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double getVisionAngle() {
    return SmartDashboard.getNumber("Angle", -1);
  }

  private double getVel() {
    int distance = (int) SmartDashboard.getNumber("Distance", -1);
    int distance1 = distance - distance % 50;
    int distance2 = distance1 + 50;
    double vel1 = dictionary.get(distance1)[0];
    double vel2 = dictionary.get(distance2)[0];
    return vel1 + ((distance - distance1) / 50.) * (vel2 - vel1);
  }

  private double getAngle() {
    int distance = (int) SmartDashboard.getNumber("Distance", -1);
    int distance1 = distance - distance % 50;
    int distance2 = distance1 + 50;
    double angle1 = dictionary.get(distance1)[1];
    double angle2 = dictionary.get(distance2)[1];
    return angle1 + ((distance - distance1) / 50.) * (angle2 - angle1);
  }
}
