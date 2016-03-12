
#include <iostream>
#include "myClass.h"


int main(int argc, char** argv)
{
  IMyClass* p = g_myClass;
  p->myFunc(47);
  p->myFunc(11);
}
