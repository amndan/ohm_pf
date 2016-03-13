
#include <iostream>
#include "myClass.h"


int main(int argc, char** argv)
{
  //IMyClass* p = g_myClass;
  
  IMyClass* p = IMyClass::create();

  p->myFunc(47);
  p->myFunc(11);
}
