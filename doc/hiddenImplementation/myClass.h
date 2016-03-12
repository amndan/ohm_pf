

class IMyClass
{
  public:
  virtual ~IMyClass(){};
  IMyClass(){};
  virtual void myFunc(int i) = 0;
};

extern IMyClass* g_myClass;
