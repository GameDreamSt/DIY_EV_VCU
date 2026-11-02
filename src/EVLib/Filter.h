
#include <cstdint>
#include <vector>

uint64_t FilterGetTimeMicroseconds();

template <typename T>
class FilterBase
{
    public:
    virtual void SetData(T data) {}
    virtual T GetData() { return T(); }
};

template <typename T>
class TimedFilter : public FilterBase<T>
{
private:
    T currentData;
    T incomingData;
    uint64_t timestampFence;
    uint64_t aheadTime;

    void RefreshTime();

public:
    TimedFilter(uint64_t milisecondsAhead);
    void SetData(T data) override;
    T GetData() override;
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

class BoolFilter : public FilterBase<bool>
{
private:
    std::vector<bool> data;
    int currentIndex;

public:
    BoolFilter(int dataCount);
    void SetData(bool newData) override;
    bool GetData() override;
};

template <typename T>
class FilterGroup : public FilterBase<T>
{
    private:
    std::vector<FilterBase<T>> filters;

    public:
    void AddFilter(FilterBase<T> filter);
    void SetData(T data) override;
    T GetData() override;
};

template<class T>
void FilterGroup<T>::AddFilter(FilterBase<T> filter)
{
    filters.push_back(filter);
}

template<class T>
void FilterGroup<T>::SetData(T data)
{
    if(filters.size() > 0)
        filters[0].SetData(data);

    for(int i = 1; i < filters.size(); i++)
    {
        filters[i].SetData(filters[i - 1].GetData());
    }
}

template<class T>
T FilterGroup<T>::GetData()
{
    if(filters.size() == 0)
        return T();
    return filters[filters.size() - 1].GetData();
}