#include"includes/BNLabelStore.h"

BNLabelStore::BNLabelStore()
{
	//eventually read from a file/csv. For now, simply put all information here
	addLabel(0,"unlabelled",255,255,255);
	addLabel(1,"book",255,0,0);
	addLabel(2,"table",0,255,0);
	addLabel(3,"mug",20,10,255);
	addLabel(4,"carton",0,255,255);
	addLabel(5,"bottle",255,255,0);
	addLabel(6,"bowl",255,125,40);
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

std::string BNLabelStore::GetNameForLabel(uint labelID)
{
	return m_labels[labelID].m_labelName;
}

//siddhant: This is slightly laughable. We need a complete re-haul of the code architecture.
// This exists because we save a labelled point cloud, which has colors corresponding to each of the labels
// so to write the point cloud to file, we can use the labelled point cloud directly, if we know the label, given a color
// this is what we are doing here.
uint BNLabelStore::GetLabelForColor(uint red, uint green, uint blue)
{
	auto labelsStartIt = m_labels.begin();
	auto labelsEndIt = m_labels.end();

	while(labelsStartIt != labelsEndIt)
	{
		if (labelsStartIt->m_color.red == red && labelsStartIt->m_color.blue == blue && labelsStartIt->m_color.green == green)
		{
			return labelsStartIt->m_labelID;
		}  
		labelsStartIt ++;
	}

	return m_labels.size();
}

uint BNLabelStore::GetSize()
{
	return m_labels.size();
}