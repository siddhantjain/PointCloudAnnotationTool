#include"includes/BNLabelStore.h"

BNLabelStore::BNLabelStore()
{
	//eventually read from a file/csv. For now, simply put all information here
	addLabel(1,"class1",255,0,0);
	addLabel(2,"class2",0,255,0);
	addLabel(3,"class3",0,0,255);
	addLabel(4,"class4",0,255,255);
	addLabel(5,"class5",255,0,0);
}

void BNLabelStore::addLabel(uint labelID,std::string labelName, uint red, uint green, uint blue)
{
	BNLabel newLabel(labelID,labelName,red,green,blue);
	m_labels.push_back(newLabel);	
}

std::vector<BNLabel>& BNLabelStore::GetLabels()
{
	return m_labels;	
}

BNLabelColor BNLabelStore::GetColorForLabel(uint labelID)
{
	return m_labels[labelID].m_color;
}