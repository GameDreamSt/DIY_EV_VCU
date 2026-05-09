
#include <vector>
#include <string>

typedef void (*cmdPtr)();

struct CommandPointer
{
  public:
    std::string name;
    cmdPtr ptr;

    CommandPointer(std::string Name, cmdPtr Ptr)
    {
        name = Name;
        ptr = Ptr;
    }
};

void InitializeSerialReader();
void TickSerialReader();
void AddCommand(CommandPointer command);
std::vector<std::string>* GetParameters();
void ToLower(std::string &str);