// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
//#include <frc/motorcontrol/PWMSparkMax.h>

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */
class Robot : public frc::TimedRobot
{
  static const int leftLeadDeviceID = 10, leftFollowDeviceID = 11, rightLeadDeviceID = 20, rightFollowDeviceID = 21;
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushed};

  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};
  frc::Joystick m_stick{0};

  static double deadband(const double input, const double threshold){
    if (std::abs(input) > std::abs(threshold)){
      return input;
    }
    return 0;
  }

public:
  Robot()
  {
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_leftLeadMotor);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_rightLeadMotor);
  }

  void RobotInit() override
  {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightLeadMotor.SetInverted(false);
    m_leftLeadMotor.SetInverted(true);

    m_leftFollowMotor.Follow(m_leftLeadMotor);
    m_rightFollowMotor.Follow(m_rightLeadMotor);
  }

  void TeleopPeriodic() override
  {
    // Drive with arcade style
    m_robotDrive.ArcadeDrive(deadband(-m_stick.GetY() * (((1 - m_stick.GetThrottle()) / 2) ), 0.05),
                            deadband(-m_stick.GetX() * 0.5, 0.05));
  }
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
