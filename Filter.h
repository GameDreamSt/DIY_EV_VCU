
#include <cstdint>

uint64_t FilterGetTimeMicroseconds();

template <typename T>
class TimedFilter
{
private:
    T currentData;
    T incomingData;
    uint64_t timestampFence;
    uint64_t aheadTime;

    void RefreshTime();

public:
    TimedFilter(uint64_t milisecondsAhead);
    void SetData(T data);
    T GetData();
};

template<class T>
void TimedFilter<T>::RefreshTime()
{
    timestampFence = FilterGetTimeMicroseconds() / 1000 + aheadTime;
}

template<class T>
TimedFilter<T>::TimedFilter(uint64_t milisecondsAhead)
{
    aheadTime = milisecondsAhead;
}

template<class T>
void TimedFilter<T>::SetData(T data)
{
    bool wasDifferent = incomingData != data;
    incomingData = data;

    if(FilterGetTimeMicroseconds() > timestampFence && wasDifferent)
    {
        RefreshTime();
        currentData = incomingData;
    }
}

template<class T>
T TimedFilter<T>::GetData()
{
    return currentData;
}