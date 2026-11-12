
#include "SerialPrint.h"
#include <cstdint>
#include <vector>

uint64_t FilterGetTimeMicroseconds();

class FilterBase
{
  public:
    virtual void SetData(bool data)
    {
        PrintSerialMessage("Using unindented virtual function!");
    }
    virtual bool GetData()
    {
        PrintSerialMessage("Using unindented virtual function!");
        return false;
    }
};

class TimedFilter : public FilterBase
{
  private:
    bool currentData;
    bool incomingData;
    float timestampFence;
    float delayTime;

  public:
    TimedFilter(float milisecondsAhead);
    void SetData(bool data);
    bool GetData();
};

class BoolFilter : public FilterBase
{
  private:
    std::vector<bool> data;
    int currentIndex;

  public:
    BoolFilter(int dataCount);
    void SetData(bool newData);
    bool GetData();
};

class FilterGroup : public FilterBase
{
  private:
    std::vector<FilterBase *> filters;

  public:
    ~FilterGroup();
    void AddFilter(FilterBase *filter);
    void SetData(bool data);
    bool GetData();
};