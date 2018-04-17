#include "common.h"

#ifndef BN_STATE_H
#define BN_STATE_H


class BNState
{
public:
    BNState();
    std::string GetState();
    void SetState(std::string);
private:
    //siddhant: this is temporary. We should ideally have some enum at least.
    std::string m_state;
};

#endif