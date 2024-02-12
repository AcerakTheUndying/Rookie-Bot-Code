// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */
class Robot : public frc::TimedRobot
{
  frc::PWMSparkMax m_leftMotorLead{10};
  frc::PWMSparkMax m_leftMotorFollow{11};
  frc::PWMSparkMax m_rightMotorLead{20};
  frc::PWMSparkMax m_rightMotorFollow{21};

  frc::DifferentialDrive m_robotDrive{
      [&](double output)
      { m_leftMotorLead.Set(output); },
      [&](double output)
      { m_rightMotorLead.Set(output); }};
  frc::Joystick m_stick{0};

public:
  Robot()
  {
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_leftMotorLead);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_rightMotorLead);
  }

  void RobotInit() override
  {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotorLead.SetInverted(true);

    // m_leftMotorLead.AddFollower(m_leftMotorFollow);
    // m_rightMotorLead.AddFollower(m_rightMotorFollow);
  }

  void TeleopPeriodic() override
  {
    // Drive with arcade style
    m_robotDrive.ArcadeDrive(-m_stick.GetY(), -m_stick.GetX());
  }
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
