
#include <iostream>
#include "../myClass.h"


class myClass : public IMyClass
{
  public:

    myClass(){};

    virtual ~myClass(){};

    void myFunc(int i)
    {
      std::cout << "this was myFunc with: " << i << std::endl;
    };
};

IMyClass* IMyClass::create()
{
  return new myClass();
}


//IMyClass* g_myClass = new myClass();

