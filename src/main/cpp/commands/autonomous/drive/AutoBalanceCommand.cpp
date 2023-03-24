#include "autonomous/commands/drive/AutoBalanceCommand.hpp"

#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>

#include "utils/JsonUtils.hpp"
#include "utils/MathHelper.hpp"

AutoBalanceCommand::AutoBalanceCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info) : m_wrapper(wrapper), m_info(info) { 
    AddRequirements({&m_wrapper.m_drive, &m_wrapper.m_leds});
}
AutoBalanceCommand::AutoBalanceCommand(AutoSubsystemWrapper& wrapper) : m_wrapper(wrapper), m_info(0, nlohmann::json_abi_v3_11_2::json()) { 
    AddRequirements({&m_wrapper.m_drive, &m_wrapper.m_leds});
}
void AutoBalanceCommand::Initialize() {
    m_old_state = m_wrapper.m_leds.GetState();
}
void AutoBalanceCommand::Execute() {
    double speed = m_wrapper.m_drive.GetRoll()/10_deg;

    speed = std::clamp(speed, -1.0, 1.0);

    m_wrapper.m_drive.Drive(speed * -0.25_mps, 0_mps, 0_rad_per_s, false, true);
}
void AutoBalanceCommand::End(bool interrupted) {
    m_wrapper.m_leds.SetState(m_old_state);
}
bool AutoBalanceCommand::IsFinished() {
    return false;
}