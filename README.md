# 2021_Romi_Color_Conundrum
Repo to store our code for the [Color Conundrum](https://wpilib.org/blog/bae-systems-mini-bot-challenge-2-color-conundrum).

![](https://images.squarespace-cdn.com/content/v1/5d4b06a67cd3580001ded283/1614822917978-I72NEESG0XGH9VFFST41/ke17ZwdGBToddI8pDm48kOqYnsiUDjsZaJqFDGXMr4gUqsxRUqqbr1mOJYKfIPR7LoDQ9mXPOjoJoqy81S2I8N_N4V1vUb5AoIIIbLZhVYy7Mythp_T-mtop-vrsUOmeInPi9iDjx9w8K4ZfjXt2doVXUrDOVQ_2ZtxCciDuvb26pkjsECF95UiDEciaBHtjCjLISwBs8eEdxAxTptZAUg/Course+1%400.25x.png?format=1500w)

![](https://images.squarespace-cdn.com/content/v1/5d4b06a67cd3580001ded283/1614822955279-NSN404I6Q9NCP2WEUQ90/ke17ZwdGBToddI8pDm48kOqYnsiUDjsZaJqFDGXMr4gUqsxRUqqbr1mOJYKfIPR7LoDQ9mXPOjoJoqy81S2I8N_N4V1vUb5AoIIIbLZhVYy7Mythp_T-mtop-vrsUOmeInPi9iDjx9w8K4ZfjXt2doVXUrDOVQ_2ZtxCciDuvb26pkjsECF95UiDEciaBHtjCjLISwBs8eEdxAxTptZAUg/Course+2%400.25x.png?format=1500w)

Submission videos:
* Teleop: _TODO_
* Auto: _TODO_

# Documentation

  * [Color Conundrum Rules update 1.1](https://wpilib.org/s/Color-Conundrum-Rules-11.pdf)
  * [Full romi hardware user manual](https://www.pololu.com/docs/0J69/all)
  * WPILib [getting started with Romi documumentation](https://docs.wpilib.org/en/latest/docs/romi-robot/index.html)

  * This project is based on:
    * [this git repo](https://github.com/bb-frc-workshops/romi-examples/tree/main/romi-trajectory-ramsete)
    * See chief delphi post: https://www.chiefdelphi.com/t/trajectory-following-with-the-romi/390505/
   

# Usage:

## Battery installation

> The correct orientation for the batteries is indicated by the battery-shaped holes in the Romi chassis as
> well as the + and - indicators in the chassis itself.

![romi batteries](https://docs.wpilib.org/en/latest/_images/assembly-batteries.png)

## Setup wifi on the romi:

https://docs.wpilib.org/en/latest/docs/romi-robot/imaging-romi.html

## Deploying code:

https://docs.wpilib.org/en/latest/docs/romi-robot/programming-romi.html#running-a-romi-program

> One aspect where a Romi project differs from a regular FRC robot project is that the code is not deployed
> directly to the Romi. Instead, a Romi project runs on your development computer, and leverages the WPILib
> simulation framework to communicate with the Romi robot.

To run a Romi program:
 * first, ensure that your Romi is powered on.
 * Connect to the WPILibPi-<number> wifi network broadcast by the Romi
 * running the code in simulation mode
 * Put the robot in `Teleoperated` or `Autonomous` modes will cause the code to execute on the romi.

## IMU calibration

The IMU is a sensor on the Romi that can tell us it's rotational position in three dimensions.
For auotonously driving the robot, We are primarily interested in using the IMUs Z axis heading data,
as this can tells us which wat the Romi is facing. For example, we can use it to drive in straight
lines, or turn to face a specific direction.

This sensor's accuracy appears to be fairly sensitive to temperature fluctuations. When it's working correctly, 
and the robot is sitting still, we should see that the robot's heading isn't changing. If the sensor calibration
is off, the sensor will report that the robot is turning, even though it's stationary. We call this error drift.
The drift can be measured and accounted for though by running a calibration routine on the Romi. 

Note that it doesn't appear this sensor can account for temperature fluctuations, so it might be best to have
the robot turned on for a few minutes so it can warm up beore running the calibration. When calibrated you
should observe ~0.01 deg/s drift. An incorrectly calibrated sensor will have ~1.0 deg/sec drift.

https://docs.wpilib.org/en/latest/docs/romi-robot/web-ui.html#imu-calibration

![imu calibration](https://docs.wpilib.org/en/latest/_images/romi-ui-imu-calibration.png)

