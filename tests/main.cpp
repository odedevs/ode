// openode_UnitTest++.cpp : Defines the entry point for the console application.
//

#include <UnitTest++.h>
#include <ode/ode.h>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>

// Suppress ODE message spam (LCP warnings) during tests.
// Error and debug handlers remain default (exit/abort) since they are ODE_NORETURN.
static void quietMessageHandler(int errnum, const char *msg, va_list ap)
{
    // Suppress LCP and other non-fatal messages during tests
    (void)errnum; (void)msg; (void)ap;
}

int main()
{
    dInitODE();
    dSetMessageHandler(quietMessageHandler);
    int res = UnitTest::RunAllTests();
    dCloseODE();
    return res;
}
