/*
Adept MobileRobots Robotics Interface for Applications (ARIA)
Copyright (C) 2004-2005 ActivMedia Robotics LLC
Copyright (C) 2006-2010 MobileRobots Inc.
Copyright (C) 2011-2014 Adept Technology

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
Adept MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
Adept MobileRobots, 10 Columbia Drive, Amherst, NH 03031; +1-603-881-7960
*/
#include "Aria.h"


/** @example wander.cpp 
 * Example using actions and range devices to implement a random wander avoiding obstacles.
 *
 *  This program will just have the robot wander around. It uses some avoidance 
 *  actions if obstacles are detected with the sonar or laser (if robot has a
 *  laser), otherwise it just has a constant forward velocity.
 * 
 *  Press Control-C or Escape keys to exit.
 *  
 * This program will work either with the MobileSim simulator or on a real
 * robot's onboard computer.  (Or use -remoteHost to connect to a wireless
 * ethernet-serial bridge.)
*/

int main(int argc, char **argv)
{
  Aria::init();
  ArArgumentParser argParser(&argc, argv);
  argParser.loadDefaultArguments();
  ArRobot robot;
  ArRobotConnector robotConnector(&argParser, &robot);
  ArLaserConnector laserConnector(&argParser, &robot, &robotConnector);

  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "Could not connect to the robot.");
    if(argParser.checkHelpAndWarnUnparsed())
    {
        // -help not given, just exit.
        Aria::logOptions();
        Aria::exit(1);
        return 1;
    }
  }


  // Trigger argument parsing
  if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(1);
    return 1;
  }

  ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);
  robot.attachKeyHandler(&keyHandler);

  puts("This program will make the robot wander around. It uses some avoidance\n"
  "actions if obstacles are detected, otherwise it just has a\n"
  "constant forward velocity.\n\nPress CTRL-C or Escape to exit.");
  
  ArSonarDevice sonar;
  robot.addRangeDevice(&sonar);

  robot.runAsync(true);

  
  // try to connect to laser. if fail, warn but continue, using sonar only
  if(!laserConnector.connectLasers())
  {
    ArLog::log(ArLog::Normal, "Warning: unable to connect to requested lasers, will wander using robot sonar only.");
  }


  // turn on the motors, turn off amigobot sounds
  robot.enableMotors();
  robot.comInt(ArCommands::SOUNDTOG, 0);

  // add a set of actions that combine together to effect the wander behavior //działa
	//general structure: robot.addAction (new Name_of_action (parameters), priority);
	//to find out more about Actions go to ~/catkin_ws/src/rosaria/local/Aria/docs/index.html

// if we're stalled we want to back up and recover
  robot.addAction(new ArActionStallRecover("stall recover", 100,50,100,30), 100); //basic values 225, 50, 150, 45
								//when_stop, cycles_to_move, speed, turn'
//slow down when near obstacle
  robot.addAction(new ArActionLimiterForwards("speed limiter near", 
						200, 200, 200), 95); // basic values 300,600,250
								//stop_distance, slow_distance, slow_speed
  // react to bumpers
  robot.addAction(new ArActionLimiterBackwards, 75);

// turn avoid very close
  robot.addAction(new ArActionAvoidFront("speed limiter far", 150,450,20), 47);	//basic values 450, 200,15
						//when_turn, speed, turn'

// turn avoid things near
  robot.addAction(new ArActionAvoidFront("speed limiter far", 300,450,10), 46);	//basic values 450, 200,15
						//when_turn, speed, turn'

// turn avoid things further away
  robot.addAction(new ArActionAvoidFront("speed limiter far", 800,450,5), 45);	//basic values 450, 200,15
						//when_turn, speed, turn'
  
//move forward
  robot.addAction(new ArActionConstantVelocity("Constant Velocity", 500), 25);



  
  // wait for robot task loop to end before exiting the program
  robot.waitForRunExit();
  
  Aria::exit(0);
  return 0;
}
