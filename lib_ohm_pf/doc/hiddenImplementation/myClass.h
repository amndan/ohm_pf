

class IMyClass
{
  public:
  virtual ~IMyClass(){};
  IMyClass(){};
  virtual void myFunc(int i) = 0;
  static IMyClass* create();
};

//extern IMyClass* g_myClass;
