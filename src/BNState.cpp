#include "includes/BNState.h"

BNState::BNState()
{
	std::cout << "Initialising state " << endl;
	m_state = "Init";
}

std::string BNState::GetState()
{
	return m_state;
}

void BNState::SetState(std::string inState)
{
	m_state = inState;
}