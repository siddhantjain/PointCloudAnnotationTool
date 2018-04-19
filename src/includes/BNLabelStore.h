#include "common.h"
#include "BNLabel.h"

#ifndef BN_LABEL_STORE_H
#define BN_LABEL_STORE_H


class BNLabelStore
{
public:
    BNLabelStore();
    std::vector<BNLabel>& GetLabels();
    BNLabelColor GetColorForLabel(uint labelID);
    std::string GetNameForLabel(uint labelID);
private:
    std::vector<BNLabel> m_labels;
    void addLabel(uint labelID,std::string labelName,uint red, uint green, uint blue);
};

#endif