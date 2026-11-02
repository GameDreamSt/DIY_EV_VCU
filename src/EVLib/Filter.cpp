
#include "Filter.h"
#include "EVTime.h"

using namespace std;

uint64_t FilterGetTimeMicroseconds()
{
    return GetTimeMicroseconds();
}

BoolFilter::BoolFilter(int dataCount)
{
    data.resize(dataCount);
}

void BoolFilter::SetData(bool newData)
{
    data[currentIndex] = newData;
    currentIndex = currentIndex++ % data.size();
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