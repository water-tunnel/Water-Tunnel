#include <stdio.h>
#include <stdlib.h>
//#include <math.h>
#using <system.dll>

using namespace System;
using namespace System::Timers;

public ref class Timer1
{
private: 
   static System::Timers::Timer^ aTimer;

public:
   static void Demo()
   {
      // Normally, the timer is declared at the class level,
      // so that it stays in scope as long as it is needed.
      // If the timer is declared in a long-running method,  
      // KeepAlive must be used to prevent the JIT compiler 
      // from allowing aggressive garbage collection to occur 
      // before the method ends. You can experiment with this
      // by commenting out the class-level declaration and 
      // uncommenting the declaration below; then uncomment
      // the GC::KeepAlive(aTimer) at the end of the method.
      //System::Timers::Timer^ aTimer;

	  printf("At 1!\n");
      // Create a new Timer with Interval set to 10 seconds.
      aTimer = gcnew System::Timers::Timer( 1000000 );

	  printf("At 2!\n");
      // Hook up the Elapsed event for the timer.
      aTimer->Elapsed += gcnew ElapsedEventHandler( Timer1::OnTimedEvent );

	  aTimer->Interval = 2000;
      aTimer->Enabled = true;      // Set the Interval to 2 seconds (2000 milliseconds).

      Console::WriteLine("Press the Enter key to exit the program.");
      Console::ReadLine();

	  printf("At 3!\n");
      // If the timer is declared in a long-running method, use
      // KeepAlive to prevent garbage collection from occurring
      // before the method ends.
      //GC::KeepAlive(aTimer);
   }


private:
   // Specify what you want to happen when the Elapsed event is 
   // raised.
   static void OnTimedEvent( Object^ source, ElapsedEventArgs^ e )
   {
      printf("Hey!!\n");
   }

};

int main()
{
	printf("Hey!!!!\n\n");
	Timer1::Demo();
	return 0;
}