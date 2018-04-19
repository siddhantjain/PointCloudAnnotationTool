#include "common.h"


#ifndef BN_LABEL_H
#define BN_LABEL_H

struct BNLabelColor
{
	uint red,green,blue;
	BNLabelColor(uint r,uint g, uint b)
	{
		red = r;
		green = g;
		blue = b;
	}
};

struct BNLabel
{
	
	BNLabel(uint labelID, std::string labelName, uint r, uint g, uint b):
	m_color(r,g,b)
	{
		m_labelID = labelID;
		m_labelName = labelName; 
	}
	uint m_labelID;
	std::string m_labelName;
	BNLabelColor m_color;
};



#endif