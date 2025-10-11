
class Timer
{
  private:
    float triggerTime;
    float currentTime;

  public:
    Timer(float fireInterval);
    bool HasTriggered();
};