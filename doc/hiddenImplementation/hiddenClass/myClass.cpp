
#include <iostream>
#include "myClass.h"


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

IMyClass* g_myClass = new myClass();

