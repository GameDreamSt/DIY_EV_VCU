
#include "Filter.h"
#include "EVTime.h"
#include "SerialPrint.h"

using namespace std;

TimedFilter::TimedFilter(float delay)
{
    delayTime = delay;
}

void TimedFilter::SetData(bool data)
{
    incomingData = data;

    if (GetTime() > timestampFence && incomingData != currentData)
    {
        timestampFence = GetTime() + delayTime;
        currentData = incomingData;
    }
}

bool TimedFilter::GetData()
{
    return currentData;
}

BoolFilter::BoolFilter(int dataCount)
{
    data.resize(dataCount);
}

void BoolFilter::SetData(bool newData)
{
    data[currentIndex] = newData;
    currentIndex = ++currentIndex % data.size();
}

bool BoolFilter::GetData()
{
    int on = 0, off = 0;
    for(int i = 0; i < data.size(); i++)
    {
        if(data[i])
            on++;
        else
            off++;
    }

    return on > off;
}

void FilterGroup::AddFilter(FilterBase* filter)
{
    filters.push_back(filter);
}

void FilterGroup::SetData(bool data)
{
    if (filters.size() > 0)
        filters[0]->SetData(data);

    for (int i = 1; i < filters.size(); i++)
    {
        filters[i]->SetData(filters[i - 1]->GetData());
    }
}

bool FilterGroup::GetData()
{
    if (filters.size() == 0)
        return false;
    return filters[filters.size() - 1]->GetData();
}

FilterGroup::~FilterGroup()
{
    for (int i = 0; i < filters.size(); i++)
    {
        delete filters[i];
    }
}